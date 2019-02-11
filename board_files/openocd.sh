#!/bin/sh
echo "Starting openocd on etags..."
sleep 1
openocd -f /usr/local/share/openocd/scripts/interface/stlink.cfg -f ./openocd_target_etag.cfg
