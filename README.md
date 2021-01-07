# osb2mp4

An [osu!](https://osu.ppy.sh) storyboard to video converter. Will be maintained until an alternative exists in lazer.

This project has been in progress on and off since the middle of September 2020. Actual time spent working so far has probably been about 5 weeks. The  `osb2mp4.py` is what I first wrote as a proof of concept.

I'll expand this readme eventually. Here are some benchmarks on my computer:

```
CPU: AMD Ryzen 5 2400G @ 3.70 GHz
Render: 1920x1080 @ 30fps
CPU usage: ~90-95%
```

| Storyboard             | time (mm:ss) |
|------------------------|-------------:|
| Alexithymia            |        40:00 |
| Double Pendulum        |        17:00 |
| world.execute(me)      |        11:00 |
| Asymmetry              |         5:12 |
| new beginnings         |         2:36 |

## Dependencies

- [OpenCV](https://www.opencv.org/releases) - download and extract somewhere accessible
- A global [ffmpeg](https://ffmpeg.org/download.html) installation

## TODO

- handle errors
- fix all edge-cases
- clean up dumb code
- optimise for performance a bit
- rewrite whole thing in opengl or something

## Compilation

Releases will probably come later. For now, just compile it yourself or ask me for a binary. These parts are Windows-specific.

```
clang .\osb2mp4.cpp -o .\osb2mp4.exe  -std=c++17 -I C:\path\to\opencv\build\include\ -L C:\path\to\opencv\build\x64\vc15\lib\ -l opencv_world451 -O3 -fopenmp
```

## Running

```
osb2mp4.exe "song-folder" "diff-name" [StartTime] [Duration]
```

`StartTime` and `Duration` are optional. The program will render the whole thing if unspecified.

Make sure to have `opencv_videoio_ffmpeg451_64.dll` and `opencv_world451.dll` in the same folder as `osb2mp4.exe`.

This spits out an `export.avi`, `audio.mp3` and a merged `video.mp4` file in the current working directory for now.
