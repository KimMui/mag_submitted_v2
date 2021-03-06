/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author: Jean Pihet
 * @author: Perry Hung
 */

#define DBG_COMP    DBG_SWITCH

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <errno.h>
#include <string.h>

#include <arch/armv7-m/byteorder.h>

#include "stm32.h"
#include "up_debug.h"
#include "tsb_switch.h"
#include "tsb_switch_driver_es2.h"

#define SWITCH_SPI_INIT_DELAY   (700)   // us

#define RXBUF_SIZE  272

struct sw_es2_priv {
    struct spi_dev_s    *spi_dev;
    unsigned int        id;
    uint8_t             *rxbuf;
};

#define LNUL        (0x00)
#define STRW        (0x01)
#define STRR        (0x02)
#define NACK        (0x04)
#define ENDP        (0x06)
#define INIT        (0x08)
#define HNUL        (0xff)

#define NCP_CPORT   (0x03)

#define CHECK_VALID_ENTRY(entry) \
    (valid_bitmask[15 - ((entry) / 8)] & (1 << ((entry)) % 8))

/* Attributes as sources of interrupts from the Unipro ports */
static uint16_t unipro_irq_attr[] = {
    TSB_DME_ENDPOINTRESETIND,
    TSB_DME_LINKSTARTUPIND,
    TSB_DME_LINKLOSTIND,
    TSB_DME_HIBERNATEENTERIND,
    TSB_DME_HIBERNATEEXITIND,
    TSB_DME_POWERMODEIND,
    TSB_DME_TESTMODEIND,
    TSB_DME_ERRORPHYIND,
    TSB_DME_ERRORPAIND,
    TSB_DME_ERRORDIND,
    0,                          // Not recommended to read
    TSB_DME_ERRORTIND,
    TSB_DME_ERRORDIND,
    TSB_DEBUGCOUNTEROVERFLOW,
    TSB_DME_LINKSTARTUPCNF,
    TSB_MAILBOX
};


/* Transfer function for NCP port */
static int es2_transfer(struct tsb_switch *sw,
                        uint8_t *tx_buf,
                        size_t tx_size,
                        uint8_t *rx_buf,
                        size_t rx_size)
{
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    unsigned int id = priv->id;
    unsigned int size, rcv_done = 0;
    bool null_rxbuf;
    int ret = OK;

    uint8_t write_header[] = {
        LNUL,
        STRW,
        NCP_CPORT,
        (tx_size & 0xFF00) >> 8,
        (tx_size & 0xFF),
    };

    uint8_t write_trailer[] = {
        ENDP,
        LNUL,
    };

    uint8_t read_header[] = {
        LNUL,
        STRR,
        NCP_CPORT,
        0,      // LENM
        0,      // LENL
        ENDP,
        LNUL
    };


    SPI_LOCK(spi_dev, true);
    SPI_SELECT(spi_dev, id, true);
    /* Write */
    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);
    // Wait write status, send NULL frames while waiting
    SPI_EXCHANGE(spi_dev, NULL, priv->rxbuf,
                 SWITCH_WAIT_REPLY_LEN + SWITCH_WRITE_STATUS_LEN);

    dbg_verbose("Write payload:\n");
    dbg_print_buf(DBG_VERBOSE, tx_buf, tx_size);
    dbg_insane("Write status:\n");
    dbg_print_buf(DBG_INSANE, priv->rxbuf,
                  SWITCH_WAIT_REPLY_LEN + SWITCH_WRITE_STATUS_LEN);

    // Make sure we use 16-bit frames
    size = sizeof write_header + tx_size + sizeof write_trailer
           + SWITCH_WAIT_REPLY_LEN + SWITCH_WRITE_STATUS_LEN;
    if (size % 2) {
        SPI_SEND(spi_dev, LNUL);
    }

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = SWITCH_WAIT_REPLY_LEN + rx_size + sizeof read_header;
        SPI_SNDBLOCK(spi_dev, read_header, sizeof read_header);
        SPI_EXCHANGE(spi_dev, NULL, priv->rxbuf, size - sizeof read_header);
        /* Make sure we use 16-bit frames */
        if (size & 0x1) {
            SPI_SEND(spi_dev, LNUL);
        }

        dbg_verbose("RX Data:\n");
        dbg_print_buf(DBG_VERBOSE, priv->rxbuf, size);

        if (!rx_buf) {
            break;
        }

        /*
         * Find the STRR and copy the response; handle other cases:
         * NACK, switch not responding.
         *
         * In some cases (e.g. wrong function ID in the NCP command) the
         * switch does respond on the command with all NULs.
         * In that case bail out with error.
         */
        uint8_t *resp_start = NULL;
        unsigned int i;
        null_rxbuf = true;

        for (i = 0; i < size; i++) {
            // Detect an all-[LH]NULs RX buffer
            if ((priv->rxbuf[i] != LNUL) && (priv->rxbuf[i] != HNUL)) {
                null_rxbuf = false;
            }
            // Check for STRR or NACK
            if (priv->rxbuf[i] == STRR) {
                // STRR found, parse the reply length and data
                resp_start = &priv->rxbuf[i];
                size_t resp_len = resp_start[2] << 8 | resp_start[3];
                memcpy(rx_buf, &resp_start[4], resp_len);
                rcv_done = 1;
                break;
            } else if (priv->rxbuf[i] == NACK) {
                // NACK found, retry the CNF read
                break;
            }
        }

        // If all NULs in RX buffer, bail out with error code
        if (null_rxbuf) {
            ret = -EIO;
            dbg_error("Switch not responding, aborting command\n");
        }

    } while (!rcv_done && !null_rxbuf);

    SPI_SELECT(spi_dev, id, false);
    SPI_LOCK(spi_dev, false);

    return ret;
}

/* Switch communication init procedure */
static int es2_init_seq(struct tsb_switch *sw)
{
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    unsigned int id = priv->id;
    const char init_reply[] = { INIT, LNUL };
    int i, rc = -1;


    SPI_LOCK(spi_dev, true);
    SPI_SELECT(spi_dev, id, true);

    // Delay needed before the switch is ready on the SPI bus
    up_udelay(SWITCH_SPI_INIT_DELAY);

    SPI_SEND(spi_dev, INIT);
    SPI_SEND(spi_dev, INIT);
    SPI_EXCHANGE(spi_dev, NULL, priv->rxbuf, SWITCH_WAIT_REPLY_LEN);

    dbg_verbose("Init RX Data:\n");
    dbg_print_buf(DBG_VERBOSE, priv->rxbuf, SWITCH_WAIT_REPLY_LEN);

    // Check for the transition from INIT to LNUL after sending INITs
    for (i = 0; i < SWITCH_WAIT_REPLY_LEN - 1; i++) {
        if (!memcmp(priv->rxbuf + i, init_reply, sizeof(init_reply)))
            rc = 0;
    }
    if (rc)
        dbg_error("%s: Failed to init the SPI link with the switch\n",
                  __func__);

    SPI_SELECT(spi_dev, id, false);
    SPI_LOCK(spi_dev, false);

    return rc;
}

/* ES2 specific interrupt handler. Clears the source of interrupt */
int es2_switch_irq_handler(struct tsb_switch *sw)
{
    uint32_t swint, swins, port_irq_status, attr_value;
    int i, j;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    do {
        // Read Switch Interrupt Status register
        if (switch_internal_getattr(sw, SWINT, &swint)) {
            dbg_error("IRQ: SWINT register read failed\n");
            return -EIO;
        }
        dbg_verbose("IRQ: SWINT=%x\n", swint);

        // Handle the Switch internal interrupts
        if (swint & TSB_INTERRUPT_SWINTERNAL) {
            if (switch_internal_getattr(sw, SWINS, &swins)) {
                dbg_error("IRQ: SWINS register read failed\n");
            }
            dbg_verbose("IRQ: Switch internal irq, SWINS=0x%04x\n", swins);

            if (swins & TSB_INTERRUPT_SPICES) {
                if (switch_internal_getattr(sw, SPICES, &attr_value)) {
                    dbg_error("IRQ: SPICES register read failed\n");
                }
                dbg_verbose("IRQ: Switch internal irq, SPICES=0x%04x\n",
                        attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI3ES) {
                if (switch_internal_getattr(sw, SPI3ES, &attr_value)) {
                    dbg_error("IRQ: SPI3ES register read failed\n");
                }
                dbg_verbose("IRQ: Switch internal irq, SPI3ES=0x%04x\n",
                            attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI4ES) {
                if (switch_internal_getattr(sw, SPI4ES, &attr_value)) {
                    dbg_error("IRQ: SPI4ES register read failed\n");
                }
                dbg_verbose("IRQ: Switch internal irq, SPI4ES=0x%04x\n",
                            attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI5ES) {
                if (switch_internal_getattr(sw, SPI5ES, &attr_value)) {
                    dbg_error("IRQ: SPI5ES register read failed\n");
                }
                dbg_verbose("IRQ: Switch internal irq, SPI5ES=0x%04x\n",
                            attr_value);
            }
        }

        // Handle external interrupts: CPorts 4 & 5
        if (swint & TSB_INTERRUPT_SPIPORT4_RX) {
            dbg_verbose("IRQ: Switch SPI port 4 RX irq\n");
        }
        if (swint & TSB_INTERRUPT_SPIPORT5_RX) {
            dbg_verbose("IRQ: Switch SPI port 5 RX irq\n");
        }

        // Handle Unipro interrupts: read the Unipro ports interrupt status
        for (i = 0; i < SWITCH_PORT_MAX; i++) {
            // If Unipro interrupt pending, read the interrupt status attribute
            if (swint & (1 << i)) {
                if (switch_dme_get(sw, i, TSB_INTERRUPTSTATUS, 0x0,
                                   &port_irq_status)) {
                    dbg_error("IRQ: TSB_INTERRUPTSTATUS(%d) register read failed\n",
                              i);
                    break;
                }
                dbg_verbose("IRQ: TSB_INTERRUPTSTATUS(%d)=0x%04x\n",
                            i, port_irq_status);

                // Read the attributes associated to the interrupt sources
                for (j = 0; j < 15; j++) {
                    if ((port_irq_status & (1 << j)) && unipro_irq_attr[j]) {
                        if (switch_dme_get(sw, i, unipro_irq_attr[j], 0x0,
                                           &attr_value)) {
                            dbg_error("IRQ: Port %d line %d attr(%04x) read failed\n",
                                      i, j, unipro_irq_attr[j]);
                        } else {
                            dbg_verbose("IRQ: Port %d line %d asserted, attr(%04x)=%04x\n",
                                        i, j, unipro_irq_attr[j], attr_value);
                        }
                    }
                }
            }
        }

    } while (swint);

    return 0;
}

/* Low level switch IRQ handler
 *
 * Posts a message in a list in order to defer the work to perform
 */
static int switch_irq_handler(int irq, void *context, void *priv)
{
    struct tsb_switch *sw = priv;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    switch_post_irq(sw);

    return 0;
}

/* Switch interrupt enable/disable */
static int es2_switch_irq_enable(struct tsb_switch *sw, bool enable)
{
    if (enable) {
        // Enable switch interrupt sources and install handler
        if (!sw->irq) {
            dbg_error("%s: no Switch context\n", __func__);
            return -EINVAL;
        }

        /*
         * Configure switch IRQ line: rising edge; install handler
         * and pass the tsb_switch struct to the handler
         */
        stm32_gpiosetevent_priv(sw->irq, true, false, true,
                                switch_irq_handler, sw);

        // Enable the switch internal interrupt sources
        if (switch_internal_setattr(sw, SWINE, SWINE_ENABLE_ALL)) {
            dbg_error("Switch SWINE register write failed\n");
            return -EIO;
        }

        // Enable the L4 interrupts
        if (switch_dme_set(sw, SWITCH_PORT_ID, TSB_INTERRUPTENABLE, 0x0,
                       TSB_L4_INTERRUPTENABLE_ALL)) {
            dbg_error("Switch INTERRUPTENABLE register write failed\n");
            return -EIO;
        }

        // Enable the SPI interrupts
        if (switch_internal_setattr(sw, SPIINTE, SPIINTE_ENABLE_ALL)) {
            dbg_error("Switch SPIINTE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPICEE, SPICEE_ENABLE_ALL)) {
            dbg_error("Switch SPICEE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI3EE, SPI3EE_ENABLE_ALL)) {
            dbg_error("Switch SPI3EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI4EE, SPI45EE_ENABLE_ALL)) {
            dbg_error("Switch SPI4EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI5EE, SPI45EE_ENABLE_ALL)) {
            dbg_error("Switch SPI5EE register write failed\n");
            return -EIO;
        }
    } else {
        // Disable switch interrupt
        stm32_gpiosetevent_priv(sw->irq, false, false, false, NULL, NULL);
    }

    return OK;
}

/* Enable/disable the interrupts for the port */
static int es2_port_irq_enable(struct tsb_switch *sw, uint8_t port_id,
                               bool enable)
{
    if (switch_dme_set(sw, port_id, TSB_INTERRUPTENABLE, 0x0,
                       enable ? TSB_INTERRUPTENABLE_ALL : 0)) {
        dbg_error("Port %d INTERRUPTENABLE register write failed\n", port_id);
        return -EIO;
    }

    return OK;
}

/* NCP commands */
static int es2_set(struct tsb_switch *sw,
                   uint8_t port_id,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_SETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, port_id, attrid, select_index, val);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portId=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, port_id, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es2_get(struct tsb_switch *sw,
                   uint8_t port_id,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_GETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                __func__, port_id, attrid, select_index);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_GETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}


static int es2_peer_set(struct tsb_switch *sw,
                        uint8_t port_id,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_PEERSETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, port_id, attrid, select_index, val);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *)  &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portId=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, port_id, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
   }
    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es2_peer_get(struct tsb_switch *sw,
                        uint8_t port_id,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_PEERGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                 __func__, port_id, attrid, select_index);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}

static int es2_lut_set(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t lut_address,
                       uint8_t dest_portid)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTSETREQ,
        unipro_portid,
        dest_portid
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, dest_portid);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): unipro_portid=%d, destPortId=%d failed: rc=%d\n",
                  __func__, unipro_portid, dest_portid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_LUTSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    dbg_verbose("fid=%02x, rc=%u, portID=%u\n",
                cnf.function_id, cnf.rc, cnf.portid);

    /* Return resultCode */
    return cnf.rc;
}

static int es2_lut_get(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t lut_address,
                       uint8_t *dest_portid)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTGETREQ,
        unipro_portid,
        NCP_RESERVED,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t dest_portid;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, *dest_portid);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): unipro_portid=%d, destPortId=%d failed: rc=%d\n",
                  __func__, unipro_portid, dest_portid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_LUTGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    *dest_portid = cnf.dest_portid;

    dbg_verbose("%s(): fid=%02x, rc=%u, portID=%u\n", __func__,
                cnf.function_id, cnf.rc, cnf.dest_portid);

    /* Return resultCode */
    return cnf.rc;
}

/**
 * @brief Dump routing table to low level console
 */
static int es2_dump_routing_table(struct tsb_switch *sw) {
    int i, j, devid, unipro_portid;
    uint8_t p = 0, valid_bitmask[16];

    dbg_info("======================================================\n");
    dbg_info("Routing table:\n");
    dbg_info(" [Port,DevId] -> [Port]\n");

    for (unipro_portid = 0; unipro_portid <= SWITCH_PORT_ID; unipro_portid++) {
        if (switch_dev_id_mask_get(sw, unipro_portid, valid_bitmask)) {
            dbg_error("%s() Failed to retrieve routing table.\n", __func__);
            return -1;
        }
        dbg_verbose("%s(): Mask ID %d\n", __func__, unipro_portid);
        dbg_print_buf(DBG_VERBOSE, valid_bitmask, sizeof(valid_bitmask));

        for (i = 0; i < 8; i++) {
            for (j = 0; j < 16; j++) {
                devid = i * 16 + j;
                if (CHECK_VALID_ENTRY(devid)) {
                    switch_lut_get(sw, unipro_portid, devid, &p);
                    dbg_info(" [%2u,%2u] -> %2u\n", unipro_portid, devid, p);
               }
            }
        }
    }

    dbg_info("======================================================\n");

    return 0;
}

static int es2_dev_id_mask_set(struct tsb_switch *sw,
                               uint8_t unipro_portid,
                               uint8_t *mask)
{
    int rc;

    struct __attribute__ ((__packed__)) req {
        uint8_t dest_deviceid;
        uint8_t portid;
        uint8_t function_id;
        uint8_t mask[16];
    } req = {
        SWITCH_DEVICE_ID,
        unipro_portid,
        NCP_SETDEVICEIDMASKREQ,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    } cnf;

    dbg_verbose("%s()\n", __func__);

    memcpy(req.mask, mask, sizeof(req.mask));

    rc = es2_transfer(sw, (uint8_t *) &req, sizeof(req),
                      (uint8_t *) &cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SETDEVICEIDMASKCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    dbg_verbose("%s(): fid=%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

static int es2_dev_id_mask_get(struct tsb_switch *sw,
                               uint8_t unipro_portid,
                               uint8_t *dst)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        unipro_portid,
        NCP_GETDEVICEIDMASKREQ,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
        uint8_t mask[16];
    } cnf;

    dbg_verbose("%s(%d)\n", __func__, unipro_portid);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_GETDEVICEIDMASKCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    memcpy(dst, &cnf.mask, sizeof(cnf.mask));

    dbg_verbose("%s(): fid=%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

static int es2_switch_attr_set(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint32_t val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRSETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHATTRSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es2_switch_attr_get(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHATTRGETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("fid=%02x, rc=%u, attr(%04x)=%04x\n",
                cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}

static int es2_switch_id_set(struct tsb_switch *sw,
                             uint8_t cportid,
                             uint8_t peer_cportid,
                             uint8_t dis,
                             uint8_t irt)
{
    int rc;

    uint8_t req[] = {
        SWITCH_DEVICE_ID,
        (dis << 2) | (irt << 0),
        NCP_SWITCHIDSETREQ,
        SWITCH_DEVICE_ID,
        cportid,            // L4 CPortID
        SWITCH_DEVICE_ID,
        peer_cportid,
        NCP_RESERVED,
        SWITCH_PORT_ID      // Source portID
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    dbg_verbose("%s: cportid: %u peer_cportid: %u dis: %u irt: %u\n",
                __func__,
                cportid,
                peer_cportid,
                dis,
                irt);

    rc = es2_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                      sizeof(struct cnf));
    if (rc) {
        dbg_error("%s() failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHIDSETCNF) {
        dbg_error("%s(): unexpected CNF\n", __func__);
        return cnf.rc;
    }

    dbg_verbose("%s(): ret=0x%02x, switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
                __func__,
                cnf.rc,
                SWITCH_DEVICE_ID,
                cportid,
                peer_cportid);

    return cnf.rc;
}

static struct tsb_switch_ops es2_ops = {
    .init_comm             = es2_init_seq,

    .set                   = es2_set,
    .get                   = es2_get,

    .peer_set              = es2_peer_set,
    .peer_get              = es2_peer_get,

    .lut_set               = es2_lut_set,
    .lut_get               = es2_lut_get,
    .dump_routing_table    = es2_dump_routing_table,

    .dev_id_mask_get       = es2_dev_id_mask_get,
    .dev_id_mask_set       = es2_dev_id_mask_set,

    .port_irq_enable       = es2_port_irq_enable,

    .switch_attr_get       = es2_switch_attr_get,
    .switch_attr_set       = es2_switch_attr_set,
    .switch_id_set         = es2_switch_id_set,

    .switch_irq_enable     = es2_switch_irq_enable,
    .switch_irq_handler    = es2_switch_irq_handler,
};

int tsb_switch_es2_init(struct tsb_switch *sw, unsigned int spi_bus)
{
    struct spi_dev_s *spi_dev;
    struct sw_es2_priv *priv;
    int rc = 0;

    dbg_info("Initializing ES2 switch...\n");

    spi_dev = up_spiinitialize(spi_bus);
    if (!spi_dev) {
        dbg_error("%s: Failed to initialize spi device\n", __func__);
        return -ENODEV;
    }

    priv = malloc(sizeof(struct sw_es2_priv));
    if (!priv) {
        dbg_error("%s: Failed to alloc the priv struct\n", __func__);
        rc = -ENOMEM;
        goto error;
    }

    priv->rxbuf = malloc(RXBUF_SIZE);
    if (!priv->rxbuf) {
        dbg_error("%s: Failed to alloc the RX buffer\n", __func__);
        rc = -ENOMEM;
        goto error;
    }

    priv->spi_dev = spi_dev;
    priv->id = SW_SPI_ID;

    sw->priv = priv;
    sw->ops = &es2_ops;

    /* Configure the SPI1 bus in Mode0, 8bits, 13MHz clock */
    SPI_SETMODE(spi_dev, SPIDEV_MODE0);
    SPI_SETBITS(spi_dev, 8);
    SPI_SETFREQUENCY(spi_dev, SWITCH_SPI_FREQUENCY);

    dbg_info("... Done!\n");

    return rc;

error:
    tsb_switch_es2_exit(sw);
    return rc;
}

void tsb_switch_es2_exit(struct tsb_switch *sw) {
    struct sw_es2_priv *priv;

    if (!sw)
        return;

    priv = sw->priv;
    if (priv)
        free(priv->rxbuf);
    free(priv);
    sw->priv = NULL;
    sw->ops = NULL;
}
