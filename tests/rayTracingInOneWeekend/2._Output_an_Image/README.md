# Chapter 2: Output an Image

for this, instead of using PPM format, I use Chris Kullas nano library for writing to  OpenExr format. This allows us to keep the full range of floating point color as computed, without losing data by converting to int.
This useful library can be found here:
https://gist.github.com/fpsunflower/e5c99116ff94114d1cbe

