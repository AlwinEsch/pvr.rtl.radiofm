# pvr.rtl.radiofm
FM Radio receiver based upon RTL-SDR as pvr addon for KODI

Add-on is usable since Kodi 15.0. RDS support comes with 16.0.

It use a from RTL-SDR supported receiver with Realtek RTL2832U as source (see http://sdr.osmocom.org/trac/wiki/rtl-sdr)
to make FM radio signal receive and send the audio stream in float format to KODI and send also RDS signal translated
to UECP in it.

On linux make sure the kernel part ```dvb_usb_rtl28xxu``` for DVB-T is not loaded, otherwise is it not usable for RTL-SDR!
