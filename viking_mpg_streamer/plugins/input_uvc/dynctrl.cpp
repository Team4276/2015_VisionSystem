/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/


#include <sys/ioctl.h>
#include <sys/time.h>
#include <getopt.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>

#include <linux/videodev2.h>

#include "../../utils.h"
#include "dynctrl.h"

/* some Logitech webcams have pan/tilt/focus controls */
#define LENGTH_OF_XU_CTR (6)
#define LENGTH_OF_XU_MAP (10)

static struct uvc_xu_control_info xu_ctrls[] = {
    {
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RELATIVE,
        0,
        4,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },
    {
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RESET,
        1,
        1,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },
    {
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_FOCUS,
        2,
        6,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },
    {
        UVC_GUID_LOGITECH_VIDEO_PIPE,
        XU_COLOR_PROCESSING_DISABLE,
        4,
        1,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },
    {
        UVC_GUID_LOGITECH_VIDEO_PIPE,
        XU_RAW_DATA_BITS_PER_PIXEL,
        7,
        1,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },
    {
        UVC_GUID_LOGITECH_USER_HW_CONTROL,
        XU_HW_CONTROL_LED1,
        0,
        3,
        UVC_CONTROL_SET_CUR | UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | UVC_CONTROL_GET_DEF | UVC_CONTROL_AUTO_UPDATE
    },

};

/* mapping for Pan/Tilt/Focus */
static struct uvc_xu_control_mapping xu_mappings[] = {
    {
        V4L2_CID_PAN_RELATIVE,
        "Pan (relative)",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RELATIVE,
        16,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_SIGNED
    },
    {
        V4L2_CID_TILT_RELATIVE,
        "Tilt (relative)",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RELATIVE,
        16,
        16,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_SIGNED
    },
    {
        V4L2_CID_PAN_RESET,
        "Pan Reset",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RESET,
        1,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_TILT_RESET,
        "Tilt Reset",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RESET,
        1,
        1,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_PANTILT_RESET_LOGITECH,
        "Pan/tilt Reset",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_PANTILT_RESET,
        8,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_FOCUS_LOGITECH,
        "Focus (absolute)",
        UVC_GUID_LOGITECH_MOTOR_CONTROL,
        XU_MOTORCONTROL_FOCUS,
        8,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_LED1_MODE_LOGITECH,
        "LED1 Mode",
        UVC_GUID_LOGITECH_USER_HW_CONTROL,
        XU_HW_CONTROL_LED1,
        8,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_LED1_FREQUENCY_LOGITECH,
        "LED1 Frequency",
        UVC_GUID_LOGITECH_USER_HW_CONTROL,
        XU_HW_CONTROL_LED1,
        8,
        16,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },
    {
        V4L2_CID_DISABLE_PROCESSING_LOGITECH,
        "Disable video processing",
        UVC_GUID_LOGITECH_VIDEO_PIPE,
        XU_COLOR_PROCESSING_DISABLE,
        8,
        0,
        V4L2_CTRL_TYPE_BOOLEAN,
        UVC_CTRL_DATA_TYPE_BOOLEAN
    },
    {
        V4L2_CID_RAW_BITS_PER_PIXEL_LOGITECH,
        "Raw bits per pixel",
        UVC_GUID_LOGITECH_VIDEO_PIPE,
        XU_RAW_DATA_BITS_PER_PIXEL,
        8,
        0,
        V4L2_CTRL_TYPE_INTEGER,
        UVC_CTRL_DATA_TYPE_UNSIGNED
    },

};

int initDynCtrls(int fd)
{
    int i = 0;
    int err = 0;
    /* try to add all controls listed above */
    for(i = 0; i < LENGTH_OF_XU_CTR; i++) {
        fprintf(stderr, "Adding control for %s\n", xu_mappings[i].name);
        if((err = xioctl(fd, UVCIOC_CTRL_ADD, &xu_ctrls[i])) < 0) {
            if(errno != EEXIST)
                perror("UVCIOC_CTRL_ADD - Error");
            else
                perror("Control exists");
        }
    }
    /* after adding the controls, add the mapping now */
    for(i = 0; i < LENGTH_OF_XU_MAP; i++) {
        fprintf(stderr, "mapping control for %s\n", xu_mappings[i].name);
        if((err = xioctl(fd, UVCIOC_CTRL_MAP, &xu_mappings[i])) < 0) {
            if(errno != EEXIST)
                perror("UVCIOC_CTRL_MAP - Error");
            else
                perror("Mapping exists");
        }
    }
    return 0;
}
