// CS5535/6 driver for SBEMU
// based on the Linux driver(cs5535audio)

#include "au_linux.h"

#ifdef AU_CARDS_LINK_GEODE

#define GEODE_DEBUG 1

#if GEODE_DEBUG
#define geodedbg(...) do { DBG_Logi("CS5535/6: "); DBG_Logi(__VA_ARGS__); } while (0)
#else
#define geodedbg(...)
#endif

#include "dmairq.h"
#include "pcibios.h"
#include "dpmi/dbgutil.h"
#include "../../drivers/cs5535audio/cs5535audio.h"

static pci_device_s geode_devices[] = {
  {"NS CS5535 Audio", 0x100B, 0x002E, 0},
  {"AMD CS5536 Audio", 0x1022, 0x2093, 0},
  {NULL,0,0,0}
};

struct geode_card_s {
  struct au_linux_card card;
};

extern int snd_cs5535audio_create(struct snd_card *card, struct pci_dev *pci);

/*extern struct snd_pcm_ops snd_m3_playback_ops;
static struct snd_pcm_ops *geode_ops = &snd_m3_playback_ops;
extern int snd_m3_probe (struct snd_card *card, struct pci_dev *pci,
                         int probe_only,
                         int spdif,
                         int enable_amp,
                         int amp_gpio);
extern irqreturn_t snd_m3_interrupt(int irq, void *dev_id);
extern void snd_m3_ac97_init (struct snd_card *card);*/

static void GEODE_close (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card = aui->card_private_data;
  if (card) {
    au_linux_close_card(&card->card);
    pds_free(card);
    aui->card_private_data = NULL;
  }
}

static void GEODE_card_info (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card = aui->card_private_data;
  char sout[100];
  sprintf(sout, "GEODE : %s (%4.4X) IRQ %u", card->card.pci_dev->device_name, card->card.pci_dev->device_id, card->card.irq);
  pds_textdisplay_printf(sout);
}

static int GEODE_adetect (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card;
  uint32_t iobase;
  int err;

  geodedbg("adetect\n");

  card = (struct geode_card_s *)pds_zalloc(sizeof(struct geode_card_s));
  if (!card)
    return 0;
  if (au_linux_find_card(aui, &card->card, geode_devices) < 0)
    goto err_adetect;
  geodedbg("PCI subsystem %4.4X:%4.4X\n", card->card.linux_pci_dev->subsystem_vendor, card->card.linux_pci_dev->subsystem_device);
  //int probe_only = aui->card_controlbits & AUINFOS_CARDCNTRLBIT_TESTCARD;
  err = snd_cs5535audio_create(card->card.linux_snd_card, card->card.linux_pci_dev);
  //err = 0;
  if (err) goto err_adetect;

#if 0 // An OPL3 will be detected, but it doesn't make any sound
  if (ioport_detect_opl(0x388)) {
    aui->fm_port = 0x388;
    aui->fm = 1;
  } else {
    if (ioport_detect_opl(0x240)) {
      aui->fm_port = 0x240;
      aui->fm = 1;
    }
  }
#endif
  /*struct snd_m3 *chip = (struct snd_m3 *)card->card.linux_snd_card->private_data;
  aui->mpu401_port = chip->iobase + 0x98;
  aui->mpu401 = 1;*/

  //if (!probe_only)
  //  snd_m3_ac97_init(card->card.linux_snd_card);

  geodedbg("GEODE : %s (%4.4X) IRQ %u\n", card->card.pci_dev->device_name, card->card.pci_dev->device_id, card->card.irq);

  return 1;

err_adetect:
  GEODE_close(aui);
  return 0;
}

static void GEODE_setrate (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card = aui->card_private_data;
  int err;

  geodedbg("setrate %u\n", aui->freq_card);
  if (aui->freq_card < 8000) {
    aui->freq_card = 8000;
  } else if (aui->freq_card > 48000) {
    aui->freq_card = 48000;
  }
  geodedbg("-> %u\n", aui->freq_card);
  aui->card_dmasize = 512;
  aui->card_dma_buffer_size = 4096; // was OK with 8KB buffer / 512B page size
  aui->dma_addr_bits = 28;
  aui->buffer_size_shift = 1;
  //err = au_linux_make_snd_pcm_substream(aui, &card->card, geode_ops);
  if (err) goto err_setrate;
  //geode_ops->prepare(card->card.pcm_substream);
  return;

 err_setrate:
  geodedbg("setrate error\n");
}

static void GEODE_start (struct mpxplay_audioout_info_s *aui)
{
  geodedbg("start\n");
  struct geode_card_s *card = aui->card_private_data;
  //geode_ops->trigger(card->card.pcm_substream, SNDRV_PCM_TRIGGER_START);
}

static void GEODE_stop (struct mpxplay_audioout_info_s *aui)
{
  geodedbg("stop\n");
  struct geode_card_s *card = aui->card_private_data;
  //geode_ops->trigger(card->card.pcm_substream, SNDRV_PCM_TRIGGER_STOP);
}

unsigned int geode_int_cnt = 0;

static long GEODE_getbufpos (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card = aui->card_private_data;
#if 0 // need to update hwptr in snd_m3_interrupt
  struct snd_m3 *chip = (struct snd_m3 *)card->card.linux_snd_card->private_data;
  struct m3_dma *s = &chip->substreams[0];
  unsigned long bufpos = s->hwptr;
#endif
unsigned long bufpos = 0;
#if 1 // just use the pointer directly
  //unsigned long bufpos = geode_ops->pointer(card->card.pcm_substream);
  //bufpos <<= 1;
#endif
#if GEODE_DEBUG > 1
  if ((geode_int_cnt % (1900)) == 0)
    geodedbg("getbufpos %u / %u\n", bufpos, aui->card_dmasize);
  //if (bufpos == aui->card_dmasize)
  //  geodedbg("getbufpos %u == dmasize\n", bufpos);
#endif
  if (bufpos < aui->card_dmasize)
    aui->card_dma_lastgoodpos = bufpos;
  return aui->card_dma_lastgoodpos;
}

static aucards_onemixerchan_s GEODE_master_vol={AU_MIXCHANFUNCS_PACK(AU_MIXCHAN_MASTER,AU_MIXCHANFUNC_VOLUME),2,{{0,255,8,0},{0,255,0,0}}};
static aucards_allmixerchan_s GEODE_mixerset[] = {
 &GEODE_master_vol,
 NULL
};

static void GEODE_writeMIXER (struct mpxplay_audioout_info_s *aui, unsigned long reg, unsigned long val)
{
  struct geode_card_s *card = aui->card_private_data;
  geodedbg("write mixer val: %X\n", val);
}

static unsigned long GEODE_readMIXER (struct mpxplay_audioout_info_s *aui, unsigned long reg)
{
  struct geode_card_s *card = aui->card_private_data;
  return 0xffff;
}

static int GEODE_IRQRoutine (struct mpxplay_audioout_info_s *aui)
{
  struct geode_card_s *card = aui->card_private_data;
  //int handled = snd_m3_interrupt(card->card.irq, card->card.linux_snd_card->private_data);
  int handled = 0;
#if GEODE_DEBUG
  if (handled) {
    if ((geode_int_cnt % (2000)) == 0) DBG_Logi("geodeirq %u\n", geode_int_cnt);
    geode_int_cnt++;
  }
#endif
  return handled;
}

one_sndcard_info GEODE_sndcard_info = {
  "CS5535/6",
  SNDCARD_LOWLEVELHAND|SNDCARD_INT08_ALLOWED,

  NULL,
  NULL,
  &GEODE_adetect,
  &GEODE_card_info,
  &GEODE_start,
  &GEODE_stop,
  &GEODE_close,
  &GEODE_setrate,

  &MDma_writedata,
  &GEODE_getbufpos,
  &MDma_clearbuf,
  &MDma_interrupt_monitor,
  &GEODE_IRQRoutine,

  //&GEODE_writeMIXER,
  //&GEODE_readMIXER,
  //&GEODE_mixerset[0],
  NULL, NULL, NULL,

  NULL,
  NULL,
  NULL, //&ioport_mpu401_write,
  NULL, //&ioport_mpu401_read,
};

#endif // AUCARDS_LINK_GEODE
