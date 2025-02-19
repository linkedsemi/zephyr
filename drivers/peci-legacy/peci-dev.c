#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/crc8_lx.h>
#include <zephyr/sys/byteorder.h>
#include <sys/types.h>
#include <zephyr/kernel.h> 
#include <zephyr/sys_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/peci-legacy.h>
#include <zephyr/pm/pm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(peci_dev, LOG_LEVEL_INF);
struct peci_ls_data {
	struct k_sem trans_sync_sem;
	struct k_sem lock;

    struct device *dev;
    struct peci_adapter *adapter;
};

long peci_dev_ioctl(struct device* dev, uint iocmd, char* umsg)
{
	struct peci_adapter *adapter;
	struct peci_ls_data *ls_data = dev->data;
	struct peci_xfer_msg *xmsg = NULL;
	struct peci_xfer_msg uxmsg;
	enum peci_cmd cmd;
	u8 *msg = NULL;
	uint msg_len;
	int ret;

	// LOG_INF("Debug %s: dev = %p\n",  __func__, dev);

    adapter = ls_data->adapter;
	cmd = iocmd;
	msg_len = sizeof(*umsg);

	// LOG_INF("Debug: Before switch\n");
	switch (cmd) {
	case PECI_CORE_CMD_XFER:
		if (msg_len != sizeof(struct peci_xfer_msg)) {
			ret = -EFAULT;
			break;
		}

        if (umsg == NULL || &uxmsg == NULL) {
            ret = -EFAULT; 
            break;
        }

        if (msg_len > sizeof(uxmsg)) {
            ret = -EFAULT;
            break;
        }

        memcpy(&uxmsg, umsg, msg_len);


		xmsg = peci_get_xfer_msg(uxmsg.tx_len, uxmsg.rx_len);
		if (!xmsg) {
			ret = -ENOMEM;
			break;
		}

        if(uxmsg.tx_len > sizeof(uxmsg.tx_buf)){
            ret = -EFAULT;
            break;
        }

        if(&uxmsg == NULL || xmsg == NULL){
            ret = -EFAULT;
            break;
        }
        memcpy(xmsg->tx_buf, uxmsg.tx_buf, uxmsg.tx_len);

		xmsg->addr = uxmsg.addr;
		xmsg->tx_len = uxmsg.tx_len;
		xmsg->rx_len = uxmsg.rx_len;

		/*
		 * Send the command and copy the results back to user space on
		 * either success or timeout to provide the completion code to
		 * the caller.
		 */
		ret = peci_command(adapter, cmd, msg_len, xmsg);
        
        if ((!ret || ret == -ETIMEDOUT) && xmsg->rx_len) {
            if (&uxmsg == NULL) {
                ret = -EFAULT;
                break;
            }

            if (xmsg->rx_len > sizeof(uxmsg.rx_buf)) {
                ret = -EFAULT; 
                break;
            }

            memcpy(uxmsg.rx_buf, xmsg->rx_buf, xmsg->rx_len);
        }

		break;

	default:
        msg = malloc(msg_len);
		memcpy(msg, umsg, msg_len);
		// LOG_INF("Debug in %s: msg in peci_dev_ioctl = %p to %p\n", __func__, (void *)msg, (void *)msg+msg_len);
		// LOG_INF("Debug in %s: umsg in peci_dev_ioctl = %p to %p\n", __func__, (void *)umsg, (void *)umsg+msg_len);
		// LOG_INF("Debug in %s: xmsg in peci_dev_ioctl = %p to %p\n", __func__, (void *)xmsg, (void *)xmsg+msg_len);
		if (msg == NULL) {
			ret = -ENOMEM;
			break;
		}

		/*
		 * Send the command and copy the results back to user space on
		 * either success or timeout to provide the completion code to
		 * the caller.
		 */
		ret = peci_command(adapter, cmd, msg_len, msg);
		memcpy(umsg, msg, msg_len);

		break;
	}

	peci_put_xfer_msg(xmsg);
	if (msg != NULL)
		free(msg);
		
	return (long)ret;
}