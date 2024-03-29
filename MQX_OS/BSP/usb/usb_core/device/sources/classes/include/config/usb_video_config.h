/**HEADER********************************************************************
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
* All Rights Reserved
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: usb_video_config.h$
* $Version :
* $Date    :
*
* Comments:
*
* @brief The file contains USB stack Video class layer api header function.
*
*****************************************************************************/

#ifndef _USB_VIDEO_CONFIG_H
#define _USB_VIDEO_CONFIG_H 1


/******************************************************************************
 * Macro's
 *****************************************************************************/
#define VIDEO_IMPLEMENT_QUEUING              (0x00)
#define MAX_VIDEO_DEVICE                     (0x01)
#define MAX_VIDEO_CLASS_CTL_EP_NUM           (0x01)
#define MAX_VIDEO_CLASS_STREAM_EP_NUM        (0x01)
#define MAX_VIDEO_CLASS_UT_NUM               (0x08)
#define MAX_VIDEO_QUEUE_ELEMS                (0x06)

#define USBCFG_VIDEO_CLASS_1_1               (1)
#define IMPLEMENT_QUEUING                    (0)
#endif
