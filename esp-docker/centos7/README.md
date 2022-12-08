# Docker files for

# BUILD
```bash
docker build -f <small/full>/Dockerfile -t <name>:<tag> ./
```
# RUN
## Persistent container
```bash
docker run -uespuser -it <name>:<tag> /bin/bash 
```
## Non-persistent container
```bash
docker run -uespuser --rm -it <name>:<tag> /bin/bash 
```
## Graphical support non-persistent container
```bash
docker run -e DISPLAY --net=host -uespuser --rm -it <name>:<tag> /bin/bash
```
## Unix X11 Forward container
* Make sure ownger and group of .Xauthority file is set to uid:gid = 1000:1000 or
* change in the dockerfile the uid of espuser to your local uid
```bash
docker run -e DISPLAY -v /tmp/.X11-unix -v $HOME/.Xauthority:/home/espuser/.Xauthority --net=host -uespuser --rm -it <name>:<tag> /bin/bash
```
# Troubleshooting
* If the build fails, try reducing the number of threads used in compilation in the Dockerfile lines 68/72 (default = 4)
