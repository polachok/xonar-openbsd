#include <sys/param.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/types.h>
#include <sys/timeout.h>
#include <uvm/uvm_param.h>
#include <sys/audioio.h>
#include <dev/audio_if.h>
#include <dev/auconv.h>
#include <dev/pci/pcidevs.h>
#include <dev/pci/pcivar.h>

#include <dev/pci/xonar.h>

int	xonar_pci_match(struct device *, void *, void *);
void	xonar_pci_attach(struct device *, struct device *, void *);
int	xonar_pci_activate(struct device *, int);
int	xonar_pci_detach(struct device *, int);
int 	xonar_intr(void *v);

struct chip_t;

struct xonar_dma {
	bus_dmamap_t dmamap;
	caddr_t addr;
	bus_dma_segment_t segments[1];
	int rsegs;
};

typedef struct xonar_t {
	struct device dev;
	struct device *audiodev;

	pci_chipset_tag_t pc;
	pcitag_t tag;
	void *ih;
	bus_space_tag_t iot;
	bus_space_handle_t ioh;
	bus_size_t map_size;
	bus_dma_tag_t dmat;
	pcireg_t pciid;
	struct xonar_dma *dma[2];
	uint32_t model;

	struct chip_t *chip;

	int anti_pop_delay;
	int output_control_gpio;

	void (*intr)(void *);
	void *intr_arg;
} xonar_t;

typedef struct chip_t {
	void *data;
	void (*init)(xonar_t *);
} chip_t;

struct wm87x6 {
	u_int16_t wm8776_regs[0x17];
	u_int16_t wm8766_regs[0x17];
};

#define XNAME(sc)			((sc)->dev.dv_xname)
#define CMI8788_READ_1(z, r)		bus_space_read_1((z)->iot, (z)->ioh, r)
#define CMI8788_READ_2(z, r)		bus_space_read_2((z)->iot, (z)->ioh, r)
#define CMI8788_READ_4(z, r)		bus_space_read_4((z)->iot, (z)->ioh, r)
#define CMI8788_WRITE_1(z, r, v)	bus_space_write_1((z)->iot, (z)->ioh, r, v)
#define CMI8788_WRITE_2(z, r, v)	bus_space_write_2((z)->iot, (z)->ioh, r, v)
#define CMI8788_WRITE_4(z, r, v)	bus_space_write_4((z)->iot, (z)->ioh, r, v)

#define SUBID_XONAR_DSX 0x8522

int xonar_init(struct xonar_t *);
int xonar_write_i2c(struct xonar_t *, uint8_t, uint8_t, uint8_t);
int xonar_write_spi(struct xonar_t *, uint8_t, uint8_t, int);

void wm8766_init(struct xonar_t *);
void wm8776_init(struct xonar_t *);
void wm87x6_init(struct xonar_t *);

struct model {
	pci_vendor_id_t pm_vid;
	pci_product_id_t pm_pid;
	const char *pm_name;
};

const struct model xonar_ids[] = {
	{ PCI_VENDOR_ASUSTEK, SUBID_XONAR_DSX, "Xonar DSX" },
};

const struct chip_t wm8766 = {
	.init = wm87x6_init,
};


int xonar_open(void *, int);
void xonar_close(void *);
int xonar_query_encoding(void *, struct audio_encoding *);
int xonar_set_params(void *, int , int , struct audio_params *, struct audio_params *);
int xonar_init_output(void *, void *, int);
void* xonar_dma_allocm(void *, int, size_t, int, int);
void xonar_dma_freem(void *, void *, int);
int xonar_getdev(void *, struct audio_device *);
int xonar_set_port(void *, mixer_ctrl_t *);
int xonar_get_port(void *, mixer_ctrl_t *);
int xonar_query_devinfo(void *handle, mixer_devinfo_t *dip);
int xonar_get_props(void *handle);
int xonar_trigger_output(void *handle, void *start, void *end, int blksize,
    void (*intr)(void *), void *arg, struct audio_params *param);
int xonar_halt_output(void *);
int xonar_trigger_input(void *, void *, void *, int,
				    void (*)(void *), void *,
				    struct audio_params *);
int xonar_halt_input(void *);
void xonar_get_default_params(void *, int, struct audio_params *);

struct audio_hw_if xonar_hw_if = {
	xonar_open, 	
	xonar_close,    
	NULL, 		/* drain */
	xonar_query_encoding,
	xonar_set_params,
	NULL, /* round_blocksize */
	NULL, /* commit_settings */
	xonar_init_output, /* init_output */
	NULL, /* init_input */
	NULL, /* start_output R */
	NULL, /* start_input R */
	xonar_halt_output, /* halt_output R */
	xonar_halt_input, /* halt_input R */
	NULL, /* sprk_ctl */
	xonar_getdev, /* getdev R */
	NULL, /* setfd */
	xonar_set_port, /* setport R */
	xonar_get_port, /* getport R */
	xonar_query_devinfo, /* query_devinfo R */
	xonar_dma_allocm, /* allocm */
	xonar_dma_freem, /* freem */
	NULL, /* round_buffersize */
	NULL, /* mappage */
	xonar_get_props, /* get_props R */
	xonar_trigger_output, /* trigger_output */
	xonar_trigger_input, /* trigger_input */
	xonar_get_default_params, /* get_default_params */
};

void
xonar_get_default_params(void *handle, int mode, struct audio_params *params)
{
	params->sample_rate = 48000;
	params->encoding = AUDIO_ENCODING_SLINEAR_LE;
	params->precision = 16;
	params->channels = 1;
	params->sw_code = NULL;
	params->factor = 1;
	params->bps = 2;
	params->msb = 1;
}

int
xonar_trigger_input(void *handle, void *start, void *end, int blksize,
    void (*intr)(void *), void *arg, struct audio_params *param)
{
	return EINVAL;
}

int xonar_halt_input(void *handle)
{
	return 0;
}

int
xonar_trigger_output(void *handle, void *start, void *end, int blksize,
    void (*intr)(void *), void *arg, struct audio_params *param)
{
	xonar_t *sc = handle;

	printf("%s: trigger_output: start %p end %p blksize %d sample_rate: %lu\n", XNAME(sc),
			start, end, blksize, param ? param->sample_rate : -1);

	sc->intr = intr;
	sc->intr_arg = arg;

	CMI8788_WRITE_4(sc, MULTICH_SIZE, ((char*)end - (char*)start) / 4 - 1);

	CMI8788_WRITE_4(sc, MULTICH_FRAG, blksize / 4 - 1);

	/* enable irq */
	CMI8788_WRITE_2(sc, IRQ_MASK,
			CMI8788_READ_2(sc, IRQ_MASK) | CHANNEL_MULTICH /*ch->play_irq_mask*/);
	/* enable dma */
	CMI8788_WRITE_2(sc, DMA_START,
			CMI8788_READ_2(sc, DMA_START) | CHANNEL_MULTICH /*ch->play_dma_start*/);

	printf("%s: irq and dma enabled!\n", XNAME(sc));


	printf("Calling set params now\n");
	xonar_set_params(handle, AUMODE_PLAY, 0,
		param, NULL);

	return 0;
}

int
xonar_halt_output(void *handle)
{
	xonar_t *sc = handle;

	/* disable dma */
	CMI8788_WRITE_2(sc, DMA_START,
			CMI8788_READ_2(sc, DMA_START) & ~CHANNEL_MULTICH /* ~ch->play_dma_start*/);
	/* disable irq */
	CMI8788_WRITE_2(sc, IRQ_MASK,
			CMI8788_READ_2(sc, IRQ_MASK) & ~CHANNEL_MULTICH /*~ch->play_irq_mask*/);

	return 0;
}

int
xonar_open(void *handle, int flags)
{
	return 0;
}

void
xonar_close(void *handle)
{
}

int xonar_get_props(void *handle)
{
	return /*AUDIO_PROP_INDEPENDENT |*/ AUDIO_PROP_FULLDUPLEX;
}

int
xonar_set_port(void *handle, mixer_ctrl_t *cp)
{
	return EINVAL;
}

int 
xonar_get_port(void *handle, mixer_ctrl_t *cp)
{
	return EINVAL;
}

int xonar_query_devinfo(void *handle, mixer_devinfo_t *dip) 
{
	return ENXIO;
}

int
xonar_query_encoding(void *handle, struct audio_encoding *fp)
{
	switch (fp->index) {
	case 0:
		strlcpy(fp->name, AudioEulinear, sizeof(fp->name));
		fp->encoding = AUDIO_ENCODING_SLINEAR_LE;
		fp->precision = 16;
		fp->flags = 0;
		break;
	case 1:
		strlcpy(fp->name, AudioEulinear, sizeof(fp->name));
		fp->encoding = AUDIO_ENCODING_SLINEAR_LE;
		fp->precision = 24;
		fp->flags = 0; // test
		break;
	case 2:
		strlcpy(fp->name, AudioEulinear, sizeof(fp->name));
		fp->encoding = AUDIO_ENCODING_SLINEAR_LE;
		fp->precision = 32;
		fp->flags = 0; // test
		break;
	case 3:
		strlcpy(fp->name, AudioEslinear, sizeof fp->name);
		fp->encoding = AUDIO_ENCODING_SLINEAR_LE;
		fp->precision = 8;
		fp->flags = 0;
		break;
	case 4:
		strlcpy(fp->name, AudioEulinear_le, sizeof fp->name);
		fp->encoding = AUDIO_ENCODING_ULINEAR_LE;
		fp->precision = 16;
		fp->flags = AUDIO_ENCODINGFLAG_EMULATED;
		break;
	case 5:
		strlcpy(fp->name, AudioEslinear_be, sizeof fp->name);
		fp->encoding = AUDIO_ENCODING_SLINEAR_BE;
		fp->precision = 16;
		fp->flags = AUDIO_ENCODINGFLAG_EMULATED;
		break;
	case 6:
		strlcpy(fp->name, AudioEulinear_be, sizeof fp->name);
		fp->encoding = AUDIO_ENCODING_ULINEAR_BE;
		fp->precision = 16;
		fp->flags = AUDIO_ENCODINGFLAG_EMULATED;
		break;
	case 8:
		strlcpy(fp->name, AudioEulinear, sizeof fp->name);
		fp->encoding = AUDIO_ENCODING_ULINEAR;
		fp->precision = 8;
		fp->flags = AUDIO_ENCODINGFLAG_EMULATED;
		break;
	default:
		return EINVAL;
	}
	fp->bps = AUDIO_BPS(fp->precision);
	fp->msb = 1; /* XXX ??? */

	return (0);
}

int 
xonar_set_params(void *handle, int setmode, int usemode,
		struct audio_params *play, struct audio_params *rec)
{
	struct xonar_t *sc = handle;
	int bits, i2s_rate;
	int i2s_bits;
	int chan_bits;

	bits = i2s_rate = i2s_bits = chan_bits = 0;

	play->sw_code = NULL;
	play->factor = 1;
	if (setmode & AUMODE_PLAY && play != NULL) {
		printf("%s: set_params: channels: %d encoding: %d, precision: %d sample_rate: %lu bps: %d\n",
			XNAME(sc), play->channels, play->encoding, play->precision, play->sample_rate, play->bps);
		switch (play->channels) {
		case 2:
			chan_bits = MULTICH_MODE_2CH;
			break;
		case 4:
			chan_bits = MULTICH_MODE_4CH;
			break;
		case 6:
			chan_bits = MULTICH_MODE_6CH;
			break;
		case 8:
			chan_bits = MULTICH_MODE_8CH;
			break;
		default:
			printf("DEFAULT @ %d", __LINE__);
			//return -EINVAL;
		}
		switch (play->bps * 8) {
		case 32:
			bits = 8;
			i2s_bits = I2S_FMT_BITS32;
			break;
		case 16:
			bits = 0;
			i2s_bits = I2S_FMT_BITS16;
			break;
		case 24:
			bits = 4;
			i2s_bits = I2S_FMT_BITS24;
			break;
		default:
			printf("DEFAULT @ %d", __LINE__);
			//return EINVAL;
		}

		switch (play->sample_rate) {
		case 32000:
			i2s_rate = I2S_FMT_RATE32;
			break;
		case 44100:
			i2s_rate = I2S_FMT_RATE44;
			break;
		case 48000:
			i2s_rate = I2S_FMT_RATE48;
			break;
		default:
			printf("DEFAULT @ %d", __LINE__);
			//return EINVAL;
		}

		printf("MULTICH_MODE = %x PLAY_FORMAT = %x I2S_MULTICH_FORMAT(bits) = %x I2S_MULTICH_FORMAT(rate) = %x\n",
			chan_bits, bits, i2s_bits, i2s_rate);

		CMI8788_WRITE_1(sc, MULTICH_MODE, 
				(CMI8788_READ_1(sc, MULTICH_MODE) &
				 ~MULTICH_MODE_CH_MASK) | chan_bits);

		CMI8788_WRITE_1(sc, PLAY_FORMAT,
				(CMI8788_READ_1(sc, PLAY_FORMAT) &
			 		~MULTICH_FORMAT_MASK) | bits);

		CMI8788_WRITE_1(sc, I2S_MULTICH_FORMAT,
				(CMI8788_READ_1(sc, I2S_MULTICH_FORMAT) &
					 ~I2S_BITS_MASK) | i2s_bits);

		CMI8788_WRITE_1(sc, I2S_MULTICH_FORMAT,
				(CMI8788_READ_1(sc, I2S_MULTICH_FORMAT) &
					 ~I2S_FMT_RATE_MASK) | i2s_rate);
		return 0; /* XXX: later */
	}
	return EINVAL;
}

int
xonar_init_output(void *handle, void *buf, int size)
{
	xonar_t *sc = handle;

	printf("%s: xonar_init_output!\n", XNAME(sc));
	CMI8788_WRITE_1(sc, CHAN_RESET,
			CMI8788_READ_1(sc, CHAN_RESET) |
			CHANNEL_MULTICH);

	DELAY(10);
	CMI8788_WRITE_1(sc, CHAN_RESET,
			CMI8788_READ_1(sc, CHAN_RESET) &
			~CHANNEL_MULTICH);


	if (sc->dma[0] == NULL) {
		printf("%s: DMA NOT INITIALIZED!!\n", XNAME(sc));
		return EINVAL;
	}	
	if (sc->dma[0]->dmamap == NULL) {
		printf("%s: DMAMAP NOT INITIALIZED!! %p\n", XNAME(sc), sc->dma[0]);
		return EINVAL;
	}	
	CMI8788_WRITE_4(sc, MULTICH_ADDR, sc->dma[0]->dmamap->dm_segs[0].ds_addr);

	return 0;
}

int
xonar_getdev(void *v, struct audio_device *dev)
{
	xonar_t *sc = v;
	int i;
	const char *model;

	for (i = 0; i < nitems(xonar_ids); i++) {
		if (sc->model == xonar_ids[i].pm_pid) {
			model = xonar_ids[i].pm_name;
			break;
		}
	}

	strlcpy(dev->name, "Xonar", MAX_AUDIO_DEV_LEN);
	snprintf(dev->version, MAX_AUDIO_DEV_LEN, "%s", model);
	strlcpy(dev->config, XNAME(sc), MAX_AUDIO_DEV_LEN);
	return 0;
}

void
xonar_dma_freem(void *hdl, void *addr, int type)
{
	xonar_t *sc = hdl;
	printf("%s: dma_free %p\n", XNAME(sc), addr);
}

void*
xonar_dma_allocm(void *hdl, int direction, size_t size, int type, int flags)
{
	xonar_t *sc = hdl;
	int error;
	struct xonar_dma *dma;
	int which;

	if (direction == AUMODE_PLAY) { /* XXX */
		which = 0;
	} else {
		which = 1;
	}

	dma = malloc(sizeof(*dma), type, flags);
	if (dma == NULL) {
		printf("%s: xonar_dma_allocm(size = %d, type = %d, flags = %d FAIL 0\n", XNAME(sc),
			(int)size, type, flags);
		return NULL;
	}

	error = bus_dmamem_alloc(sc->dmat, size, 4 /* alignment */, 0x1000000 /* boundary */,
			dma->segments, 1 /* nsegs */, &dma->rsegs /* rsegs */, BUS_DMA_NOWAIT /* flags */); 
	if (error) {
		printf("%s: xonar_dma_allocm(size = %d, type = %d, flags = %d FAIL 1\n", XNAME(sc),
			(int)size, type, flags);
		return NULL; /* XXX */
	}
	error = bus_dmamem_map(sc->dmat, dma->segments, 1, size, &dma->addr, flags | BUS_DMA_COHERENT);
	if (error) {
		printf("%s: xonar_dma_allocm(size = %d, type = %d, flags = %d FAIL 2\n", XNAME(sc),
			(int)size, type, flags);
		return NULL; /* XXX */
	}
	error = bus_dmamap_create(sc->dmat, size, 1, size, 0, BUS_DMA_NOWAIT, &dma->dmamap);
	if (error) {
		printf("%s: xonar_dma_allocm(size = %d, type = %d, flags = %d FAIL 3\n", XNAME(sc),
			(int)size, type, flags);
		return NULL; /* XXX */
	}
	error = bus_dmamap_load(sc->dmat, dma->dmamap, dma->addr, size, NULL, flags);
	if (error) {
		printf("%s: xonar_dma_allocm(size = %d, type = %d, flags = %d FAIL 4\n", XNAME(sc),
				(int)size, type, flags);
	}

	sc->dma[which] = dma;
	printf("%s: xonar_dma_allocm(DIRECTION = %d, size = %d, type = %d, flags = %d allocated OK, addr: %p", XNAME(sc), which,
			(int)size, type, flags, sc->dma[which]);

	return dma->addr;
}


int
xonar_write_i2c(xonar_t *sc, uint8_t codec_num, uint8_t reg,
		uint8_t data)
{
	int count = 50;

	/* Wait for it to stop being busy */
	while ((CMI8788_READ_2(sc, I2C_CTRL) & TWOWIRE_BUSY) && (count > 0)) {
		DELAY(10);
		count--;
	}
	if (count == 0) {
		printf("%s: i2c timeout", XNAME(sc));
		return EIO;
	}

	/* first write the Register Address into the MAP register */
	CMI8788_WRITE_1(sc, I2C_MAP, reg);

	/* now write the data */
	CMI8788_WRITE_1(sc, I2C_DATA, data);

	/* select the codec number to address */
	CMI8788_WRITE_1(sc, I2C_ADDR, codec_num);
	DELAY(100);

	return 1;
}

int
xonar_write_spi(xonar_t *sc, uint8_t codec_num, uint8_t reg, int val)
{
	int latch, shift, count;
	unsigned int tmp;

	/* check if SPI is busy */
	count = 10;
	while ((CMI8788_READ_1(sc, SPI_CONTROL) & 0x1) && count-- > 0) {
		DELAY(10);
	}

	switch (sc->model) {
	case SUBID_XONAR_DS:
	case SUBID_XONAR_DSX:
		shift = 9;
		latch = 0;
		break;
	default:
		shift = 8;
		latch = 0x80;
	}

	/* 2 byte data/reg info to be written */
	tmp = val;
	tmp |= (reg << shift);

	/* write 2-byte data values */
	CMI8788_WRITE_1(sc, SPI_DATA, tmp & 0xff);
	CMI8788_WRITE_1(sc, SPI_DATA + 1, (tmp >> 8) & 0xff);

	/* Latch high, clock=160, Len=2byte, mode=write */
	tmp = (CMI8788_READ_1(sc, SPI_CONTROL) & ~0x7E) | latch | 0x1;

	/* now address which codec you want to send the data to */
	tmp |= (codec_num << 4);

	/* send the command to write the data */
	CMI8788_WRITE_1(sc, SPI_CONTROL, tmp);

	return 0;
}

void wm87x6_init(xonar_t *sc)
{
	chip_t *chip = sc->chip;

	chip->data = malloc(sizeof(struct wm87x6), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (!chip->data) {
		printf("%s: can't allocate memory\n", XNAME(sc));
		return;
	}
}

int 
xonar_pci_match(struct device *parent, void *match, void *aux)
{
	struct pci_attach_args *pa = aux;
	pci_chipset_tag_t pc = pa->pa_pc;
	pcireg_t subid;
	int i;

	subid = pci_conf_read(pc, pa->pa_tag, PCI_SUBSYS_ID_REG);

	if (PCI_VENDOR(pa->pa_id) != PCI_VENDOR_CMI ||
		PCI_PRODUCT(pa->pa_id) != PCI_PRODUCT_CMI_CMI8788) {
		return 0;
	}

	for (i = 0; i < nitems(xonar_ids); i++) {
		if (PCI_VENDOR(subid) == xonar_ids[i].pm_vid &&
				PCI_PRODUCT(subid) == xonar_ids[i].pm_pid)
			return 1;
	}
	return 0;
}

int
xonar_init(struct xonar_t *sc)
{
	uint16_t sVal;
	uint16_t sDac;
	uint8_t bVal;
	int count;

	/* Init CMI controller */
	sVal = CMI8788_READ_2(sc, CTRL_VERSION);
	if (!(sVal & CTRL_VERSION2)) {
		bVal = CMI8788_READ_1(sc, MISC_REG);
		bVal |= MISC_PCI_MEM_W_1_CLOCK;
		CMI8788_WRITE_1(sc, MISC_REG, bVal);
	}
	bVal = CMI8788_READ_1(sc, FUNCTION);
	bVal |= FUNCTION_RESET_CODEC;
	CMI8788_WRITE_1(sc, FUNCTION, bVal);

	/* set up DAC related settings */
	sDac = I2S_MASTER | I2S_FMT_RATE48 | I2S_FMT_LJUST | I2S_FMT_BITS16;

	switch (sc->model) {
	case SUBID_XONAR_D1:
	case SUBID_XONAR_DX:
	case SUBID_XONAR_D2:
	case SUBID_XONAR_D2X:
	case SUBID_XONAR_STX:
	case SUBID_XONAR_DS:
	case SUBID_XONAR_DSX:
		/* Must set master clock. */
		sDac |= XONAR_MCLOCK_256;
		break;
	case SUBID_XONAR_ST:
		sDac |= XONAR_MCLOCK_512;
	}
	CMI8788_WRITE_2(sc, I2S_MULTICH_FORMAT, sDac);
	CMI8788_WRITE_2(sc, I2S_ADC1_FORMAT, sDac);
	CMI8788_WRITE_2(sc, I2S_ADC2_FORMAT, sDac);
	CMI8788_WRITE_2(sc, I2S_ADC3_FORMAT, sDac);

	/* setup routing regs with default values */
	CMI8788_WRITE_2(sc, PLAY_ROUTING, 0xE400);
	CMI8788_WRITE_1(sc, REC_ROUTING, 0x00);
	CMI8788_WRITE_1(sc, REC_MONITOR, 0x00);
	CMI8788_WRITE_1(sc, MONITOR_ROUTING, 0xE4);

	/* AC97 dances. Who needs it anyway? */
	/* Cold reset onboard AC97 */
	CMI8788_WRITE_2(sc, AC97_CTRL, AC97_COLD_RESET);
	count = 100;
	while ((CMI8788_READ_2(sc, AC97_CTRL) & AC97_STATUS_SUSPEND) && (count--))
	{
		CMI8788_WRITE_2(sc, AC97_CTRL,
				(CMI8788_READ_2(sc, AC97_CTRL)
				 & ~AC97_STATUS_SUSPEND) | AC97_RESUME);

		DELAY(100);
	}

	if (!count)
		printf("%s: AC97 not ready\n", XNAME(sc));

	sVal = CMI8788_READ_2(sc, AC97_CTRL);

	/* check if there's an onboard AC97 codec */
	if (sVal & AC97_CODEC0)
		printf("%s: AC97 codec0 found\n", XNAME(sc));
	/* check if there's an front panel AC97 codec */
	if (sVal & AC97_CODEC1)
		printf("%s: AC97 codec1 found\n", XNAME(sc));

	switch (sc->model) {
	case SUBID_XONAR_STX:
		sc->anti_pop_delay = 800;
		sc->output_control_gpio = GPIO_PIN0;
		CMI8788_WRITE_1(sc, FUNCTION,
				CMI8788_READ_1(sc, FUNCTION) | FUNCTION_2WIRE);

		CMI8788_WRITE_2(sc, GPIO_CONTROL,
				CMI8788_READ_2(sc, GPIO_CONTROL) | 0x018F);

		CMI8788_WRITE_2(sc, GPIO_DATA,
				CMI8788_READ_2(sc, GPIO_DATA) |
				GPIO_PIN0 | GPIO_PIN4 | GPIO_PIN8);

		CMI8788_WRITE_2(sc, I2C_CTRL,
				CMI8788_READ_2(sc, I2C_CTRL) |
				TWOWIRE_SPEED_FAST);

		//pcm1796_init(sc);
		break;
	case SUBID_XONAR_DS:
	case SUBID_XONAR_DSX:
		/* GPIO 8 = 1 output enabled 0 mute */
		/* GPIO 7 = 1 lineout enabled 0 mute */
		/* GPIO 6 = 1 mic select 0 line-in select */
		/* GPIO 4 = 1 FP Headphone plugged in */
		/* GPIO 3 = 1 FP Mic plugged in */

		/* setup for spi communication mode */
		CMI8788_WRITE_1(sc, FUNCTION, (CMI8788_READ_1(sc, FUNCTION) & ~FUNCTION_2WIRE)|0x32);
		/* setup the GPIO direction */
		CMI8788_WRITE_2(sc, GPIO_CONTROL, CMI8788_READ_2(sc, GPIO_CONTROL) | 0x1D0);
		/* setup GPIO Pins */
		CMI8788_WRITE_2(sc, GPIO_DATA, CMI8788_READ_2(sc, GPIO_DATA) | 0x1D0);
#if 1 /* XXX FIXME */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0x17, 0x1); /* reset */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0x7, 0x90); /* dac control */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0x8, 0); /* unmute */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0xC, 0x22 ); /* powerdown hp */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0xD, 0x8); /* powerdown hp */
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0xA, 0x1); /* LJust/16bit*/
		xonar_write_spi(sc, XONAR_DS_FRONTDAC, 0xB, 0x1); /* LJust/16bit*/

		xonar_write_spi(sc, XONAR_DS_SURRDAC, 0x1f, 1); /* reset */
		xonar_write_spi(sc, XONAR_DS_SURRDAC, 0x3, 0x1|0x20); /* LJust/24bit*/
#endif
		break;

	}

	/* check if MPU401 is enabled in MISC register */
	if (CMI8788_READ_1 (sc, MISC_REG) & MISC_MIDI)
		printf("%s: MPU401 found\n", XNAME(sc));

	return (0);

}

int
xonar_intr(void *v)
{
	xonar_t *sc = v;
	uint32_t intstat;

	mtx_enter(&audio_lock);
	if ((intstat = CMI8788_READ_2(sc, IRQ_STAT)) == 0) {
		mtx_leave(&audio_lock);
		return 0;
	}

	if ((intstat & CHANNEL_MULTICH)) {
		CMI8788_WRITE_2(sc, IRQ_MASK,
				CMI8788_READ_2(sc, IRQ_MASK) & ~CHANNEL_MULTICH);
		CMI8788_WRITE_2(sc, IRQ_MASK,
				CMI8788_READ_2(sc, IRQ_MASK) | CHANNEL_MULTICH);

		sc->intr(sc->intr_arg);
	}
	mtx_leave(&audio_lock);

	return 1;
}

#define PCIR_BAR(_x) (0x10 + (_x) * 4)
void
xonar_pci_attach(struct device *parent, struct device *self, void *aux)
{
	xonar_t *sc;
	struct pci_attach_args *pa;
	pcireg_t v, subid;
	//uint8_t reg;
	pci_intr_handle_t ih;
	const char *interrupt_str;

	sc = (xonar_t*)self;
	pa = aux;

	sc->dmat = pa->pa_dmat;

	pci_set_powerstate(pa->pa_pc, pa->pa_tag, PCI_PMCSR_STATE_D0);

	v = pci_conf_read(pa->pa_pc, pa->pa_tag, PCIR_BAR(0));
	v &= PCI_MAPREG_TYPE_MASK;
	if (pci_mapreg_map(pa, PCIR_BAR(0), v, 0, 
			   &sc->iot, &sc->ioh, NULL, &sc->map_size, 0)) {
		printf(": can't map device i/o space\n");
		return;
	}
	sc->pc = pa->pa_pc;
	sc->tag = pa->pa_tag;
	sc->pciid = pa->pa_id;
	subid = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_SUBSYS_ID_REG);
	sc->model = PCI_PRODUCT(subid);

	/* interrupt */
	if (pci_intr_map_msi(pa, &ih) && pci_intr_map(pa, &ih)) {
		printf(": can't map interrupt\n");
		return;
	}
	interrupt_str = pci_intr_string(pa->pa_pc, ih);
	sc->ih = pci_intr_establish(pa->pa_pc, ih, IPL_AUDIO | IPL_MPSAFE,
	    xonar_intr, sc, sc->dev.dv_xname);
	if (sc->ih == NULL) {
		printf(": can't establish interrupt");
		if (interrupt_str != NULL)
			printf(" at %s", interrupt_str);
		printf("\n");
		return;
	}
	printf(": %s: sc->dev=%p\n", interrupt_str, &sc->dev);
	xonar_init(sc);

	memset(sc->dma, 0, sizeof(sc->dma));

	sc->audiodev = audio_attach_mi(&xonar_hw_if, sc, &sc->dev);
}

int
xonar_pci_detach(struct device *self, int flags)
{
	printf("xonar detach\n");
	return 0;
}

int
xonar_pci_activate(struct device *self, int act)
{
	printf("xonar activate\n");
	return 0;
}

struct cfattach xonar_ca = {
	sizeof(xonar_t), xonar_pci_match, xonar_pci_attach,
	xonar_pci_detach, xonar_pci_activate
};

struct cfdriver xonar_cd = {
	NULL, "xonar", DV_DULL
};
