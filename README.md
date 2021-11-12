# osb2mp4

An [osu!](https://osu.ppy.sh) storyboard to video converter. Will be maintained until an alternative exists in lazer.

This project has been in progress on and off since the middle of September 2020. Actual time spent working so far has probably been about 6 weeks.

I'll expand this readme eventually. Here are some benchmarks on my computer:

```
CPU: AMD Ryzen 5 2400G @ 3.70 GHz
Render: 1920x1080 @ 30fps
CPU usage: ~90-95%
```

| Storyboard             | time (mm:ss) |
|------------------------|-------------:|
| [Alexithymia](https://osu.ppy.sh/beatmapsets/1054045)            |        40:00 |
| [Double Pendulum](https://osu.ppy.sh/beatmapsets/695053)        |        17:00 |
| [world.execute(me)](https://osu.ppy.sh/beatmapsets/470977)      |        11:00 |
| [Asymmetry](https://osu.ppy.sh/beatmapsets/310499)              |         5:12 |
| [new beginnings](https://osu.ppy.sh/beatmapsets/1011011)         |         2:36 |

## TODO

- handle errors
- fix all edge-cases
- clean up dumb code
- optimise for performance a bit
- rewrite whole thing in opengl or something

## Running

```
Usage: osb2mp4.exe song_folder [options]

options:
 -s, --start-time time          start time in ms (default: automatic)
 -e, --end-time time            end time in ms (default: automatic)
 -d, --duration time            duration in ms (default: automatic)
 -o, --output time              output video name (default: video.mp4)
 -diff, --difficulty filename   difficulty file name (default: first found)
 -w, --width pixels             video width (default: 1920)
 -h, --height pixels            video height (default: 1080)
 -f, --frame-rate fps           video frame rate (default: 30)
 -v, --volume volume            overall volume from 0 to 100 (default: 100)
 -mv, --music-volume volume     music volume from 0 to 100 (default: 100)
 -ev, --effect-volume volume    effect volume from 0 to 100, i.e. samples (default:
                                100)
 -dim, --background-dim dim     background dim value from 0 to 100 (default: 0)
 -ar, --respect-aspect-ratio    change to 4:3 aspect ratio if WidescreenStoryboard
                                is disabled in the difficulty file
 -fail, --show-fail-layer       show the fail layer instead of the pass layer
 -keep, --keep-temp-files       don't delete temporary files (temp.mp3 & temp.avi)
 -z, --zoom factor              zoom factor to use when rendering, useful for checking
                                out-of-bounds sprites (default: 1)
```

## Dependencies

- [OpenCV](https://www.opencv.org/releases) - download and extract somewhere accessible
- A global [FFmpeg](https://ffmpeg.org/download.html) installation

## Compilation

On Windows, you can compile with clang (you can get it by installing [LLVM](https://releases.llvm.org/download.html)) by running the included `make.ps1` script. Be sure to fill in the templated variables at the top of the file.

When running, make sure to have `opencv_videoio_ffmpeg451_64.dll` and `opencv_world451.dll` in the same folder as `osb2mp4.exe`.
