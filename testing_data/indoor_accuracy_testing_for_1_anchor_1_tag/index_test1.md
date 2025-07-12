# TEST 1
Indoor anchor 4 connected using Li-Ion battery 3.7V , Placed 100cm apart , Initiator powered using j9.
Orientation facing each other and antenna vertical , clear line of sight

### sit_config.c params
```
dwt_config_t sit_device_config = {
    9,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    DWT_SFD_DW_8,     /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};
```

### Timeouts
```
#define CPU_PROCESSING_TIME 400
#define POLL_TX_TO_RESP_RX_DLY_UUS_T (350 + CPU_PROCESSING_TIME)
#define RESP_RX_TO_FINAL_TX_DLY_UUS_T (350 + CPU_PROCESSING_TIME)
#define POLL_RX_TO_RESP_TX_DLY_UUS_T 920
#define RESP_TX_TO_FINAL_RX_DLY_UUS_T 620
#define RESP_RX_TIMEOUT_UUS_T 1150
#define FINAL_RX_TIMEOUT_T 1200

```
