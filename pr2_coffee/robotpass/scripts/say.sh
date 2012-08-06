#!/bin/bash

echo $1 > /tmp/tts_buf
text2wave -o /tmp/tts_buf.wav /tmp/tts_buf && aplay /tmp/tts_buf.wav 
rm /tmp/tts_buf /tmp/tts_buf.wav

