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
 * @brief: Functions and definitions for interface block management.
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>

#include "stm32.h"
#include "up_debug.h"
#include "interface.h"

#define POWER_OFF_TIME_IN_US            (500000)
#define WAKEOUT_PULSE_DURATION_IN_US    (100000)

static struct interface **interfaces;
static unsigned int nr_interfaces;


/**
 * @brief Configure all the voltage regulators associated with an interface
 * to their default states.
 * @param iface interface to configure
 */
static int interface_config(struct interface *iface) {
    unsigned int i;
    int rc = 0;

    dbg_verbose("Configuring interface %s.\n",
            iface->name ? iface->name : "unknown");

    for (i = 0; i < iface->nr_vregs; i++) {
        if (stm32_configgpio(iface->vregs[i].gpio) < 0) {
            dbg_error("%s: Failed to configure vregs pins for interface %s\n",
                      __func__, iface->name ? iface->name : "unknown");
            // Let other pins to be configured
            rc = -1;
        }
    }

    /*
     * Configure WAKEOUT as input, floating so that it does not interfere
     * with the wake and detect input pin
     */
    if (iface->wake_out) {
        if (stm32_configgpio(iface->wake_out | GPIO_INPUT) < 0) {
            dbg_error("%s: Failed to configure WAKEOUT pin for interface %s\n",
                      __func__, iface->name ? iface->name : "unknown");
            rc = -1;
        }
    }

    iface->power_state = false;

    return rc;
}


/**
 * @brief Turn on the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_enable(struct interface *iface) {
    unsigned int i;

    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Enabling interface %s.\n",
                iface->name ? iface->name : "unknown");

    for (i = 0; i < iface->nr_vregs; i++) {
        stm32_gpiowrite(iface->vregs[i].gpio, iface->vregs[i].active_high);
        up_udelay(iface->vregs[i].hold_time);
    }

    /* Update state */
    iface->power_state = true;

    return 0;
}


/**
 * @brief Turn off the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_disable(struct interface *iface) {
    unsigned int i;

    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Disabling interface %s.\n",
                iface->name ? iface->name : "unknown");


    for (i = 0; i < iface->nr_vregs; i++) {
        stm32_gpiowrite(iface->vregs[i].gpio, !iface->vregs[i].active_high);
    }

    /* Update state */
    iface->power_state = false;

    return 0;
}


/*
 * @brief Generate a WAKEOUT signal to wake-up/power-up modules.
 * If assert is true, keep the WAKEOUT lines asserted.
 *
 * The corresponding power supplies must already be enabled.
 */
int interface_generate_wakeout(struct interface *iface, bool assert)
{
    int rc;

    if (!iface) {
        return -ENODEV;
    }

   /*
    * Assert the WAKEOUT line on the interfaces in order to power up the
    * modules.
    * When the WAKEOUT signal is de-asserted the bridges have to assert
    * the PS_HOLD signal asap in order to stay powered up.
    */
    dbg_verbose("Generating WAKEOUT on interface %s.\n",
                iface->name ? iface->name : "unknown");

    if (iface->wake_out) {
        rc = stm32_configgpio(iface->wake_out | GPIO_OUTPUT | GPIO_OUTPUT_SET);
        if (rc < 0) {
            dbg_error("%s: Failed to assert WAKEOUT pin for interface %s\n",
                      __func__, iface->name ? iface->name : "unknown");
            return rc;
        }

        if (!assert) {
            /* Wait for the bridges to react */
            usleep(WAKEOUT_PULSE_DURATION_IN_US);

            /* De-assert the lines */
            rc = stm32_configgpio(iface->wake_out | GPIO_INPUT);
            if (rc < 0) {
                dbg_error("%s: Failed to de-assert WAKEOUT pin for interface %s\n",
                          __func__, iface->name ? iface->name : "unknown");
                return rc;
            }
        }
    }

    return 0;
}


/*
 * @brief Get interface power supply state, or false if no interface is supplied
 */
bool interface_get_pwr_state(struct interface *iface)
{
    if (!iface) {
        return false;
    }

    return iface->power_state;
}

/**
 * @brief Get the interface struct from the index, as specified in the MDK.
 *        Index 0 is for the first interface (aka 'A').
 * @returns: interface* on success, NULL on error
 */
struct interface* interface_get(uint8_t index)
{
    if ((!interfaces) || (index >= nr_interfaces))
        return NULL;

    return interfaces[index];
}

/**
 * @brief Given a table of interfaces, initialize and enable all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @returns: 0 on success, <0 on error
 * @sideeffects: leaves interfaces powered off on error.
 */
int interface_init(struct interface **ints, size_t nr_ints) {
    unsigned int i;
    int rc;
    int fail = 0;

    dbg_info("Initializing all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_config(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to configure interface %s\n", interfaces[i]->name);
            fail = 1;
            /* Continue configuring remaining interfaces */
            continue;
        }
    }

    if (fail) {
        return -1;
    }

    /* Let everything settle for a good long while.*/
    up_udelay(POWER_OFF_TIME_IN_US);

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_pwr_enable(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to enable interface %s\n", interfaces[i]->name);
            interface_exit();
            return rc;
        }
        rc = interface_generate_wakeout(interfaces[i], true);
        if (rc < 0) {
            dbg_error("Failed to generate wakeout on interface %s\n",
                      interfaces[i]->name);
            interface_exit();
            return rc;
        }
    }

    return 0;
}


/**
 * @brief Disable all associated power supplies. Must have been previously
 * configured with interface_init()
 */
void interface_exit(void) {
    unsigned int i;
    int rc;

    dbg_info("Disabling all interfaces\n");

    if (!interfaces) {
        return;
    }

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_pwr_disable(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to disable interface %s\n", interfaces[i]->name);
            /* Continue turning off the rest even if this one failed */
            continue;
        }
    }

    interfaces = NULL;
    nr_interfaces = 0;
}
