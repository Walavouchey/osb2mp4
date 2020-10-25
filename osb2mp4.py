# i love debugging this
import sys
import os
import json
import math
from PIL import Image, ImageDraw
from copy import copy
from time import perf_counter as pt
import numpy as np
from blend_modes import addition as additive
import subprocess as sp
import cv2 as cv
from progress.bar import Bar

t = pt()
t2 = pt()

def applyEasing(easing, t):
    def r(f, t):
        return 1 - f(1 - t)
    def io(f, t):
        return 0.5 * (f(2 * t) if t < 0.5 else (2 - f(2 - 2 * t)))

    Linear = lambda x : x
    InQuad = lambda x : x**2
    OutQuad = lambda x : r(InQuad, x)
    Out = lambda x : OutQuad(x)
    In = lambda x : InQuad(x)
    InOutQuad = lambda x : io(InQuad, x)
    InCubic = lambda x : x**3
    OutCubic = lambda x : r(InCubic, x)
    InOutCubic = lambda x : io(InCubic, x)
    InQuart = lambda x : x**4
    OutQuart = lambda x : r(InQuart, x)
    InOutQuart = lambda x : io(InQuart, x)
    InQuint = lambda x : x**5
    OutQuint = lambda x : r(InQuint, x)
    InOutQuint = lambda x : io(InQuint, x)
    InSine = lambda x : 1 - math.cos(x * math.pi / 2)
    OutSine = lambda x : r(InSine, x)
    InOutSine = lambda x : io(InSine, x)
    InExpo = lambda x : 2**(10 * (x - 1))
    OutExpo = lambda x : r(InExpo, x)
    InOutExpo = lambda x : io(InExpo, x)
    InCirc = lambda x : 1 - math.sqrt(1 - x * x)
    OutCirc = lambda x : r(InCirc, x)
    InOutCirc = lambda x : io(InCirc, x)
    OutElastic = lambda x : 2**(-10 * x) * math.sin((x - 0.075) * (2 * math.pi) / 0.3) + 1
    InElastic = lambda x : r(OutElastic, x)
    OutElasticHalf = lambda x : 2**(-10 * x) * math.sin((0.5 * x - 0.075) * (2 * math.pi) / 0.3) + 1
    OutElasticQuarter = lambda x : 2**(-10 * x) * math.sin((0.25 * x - 0.075) * (2 * math.pi) / 0.3) + 1
    InOutElastic = lambda x : io(InElastic, x)
    InBack = lambda x : x**2 * ((1.70158 + 1) * x - 1.70158)
    OutBack = lambda x : r(InBack, x)
    InOutBack = lambda x : io(InBack, x)
    OutBounce = lambda x : 7.5625 * x**2 if x < 1 / 2.75 else 7.5625 * (x - (1.5 / 2.75)) * (x - (1.5 / 2.75)) + .75 if x < 2 / 2.75 else 7.5625 * (x - (2.25 / 2.75)) * (x - (2.25 / 2.75)) + .9375 if x < 2.5 / 2.75 else 7.5625 * (x - (2.625 / 2.75)) * (x - (2.625 / 2.75)) + .984375
    InBounce = lambda x : r(OutBounce, x)
    InOutBounce = lambda x : io(OutBounce, x)
    
    if easing == 0:
        return Linear(t)
    elif easing == 1:
        return Out(t)
    elif easing == 2:
        return In(t)
    elif easing == 3:
        return InQuad(t)
    elif easing == 4:
        return OutQuad(t)
    elif easing == 5:
        return InOutQuad(t)
    elif easing == 6:
        return InCubic(t)
    elif easing == 7:
        return OutCubic(t)
    elif easing == 8:
        return InOutCubic(t)
    elif easing == 9:
        return InQuart(t)
    elif easing == 10:
        return OutQuart(t)
    elif easing == 11:
        return InOutQuart(t)
    elif easing == 12:
        return InQuint(t)
    elif easing == 13:
        return OutQuint(t)
    elif easing == 14:
        return InOutQuint(t)
    elif easing == 15:
        return InSine(t)
    elif easing == 16:
        return OutSine(t)
    elif easing == 17:
        return InOutSine(t)
    elif easing == 18:
        return InExpo(t)
    elif easing == 19:
        return OutExpo(t)
    elif easing == 20:
        return InOutExpo(t)
    elif easing == 21:
        return InCirc(t)
    elif easing == 22:
        return OutCirc(t)
    elif easing == 23:
        return InOutCirc(t)
    elif easing == 24:
        return InElastic(t)
    elif easing == 25:
        return OutElastic(t)
    elif easing == 26:
        return OutElasticHalf(t)
    elif easing == 27:
        return OutElasticQuarter(t)
    elif easing == 28:
        return InOutElastic(t)
    elif easing == 29:
        return InBack(t)
    elif easing == 30:
        return OutBack(t)
    elif easing == 31:
        return InOutBack(t)
    elif easing == 32:
        return InBounce(t)
    elif easing == 33:
        return OutBounce(t)
    elif easing == 34:
        return InOutBounce(t)

def k(keyframes, time):
    keyframe = None
    endKeyframe = None
    for i, k in enumerate(keyframes):
        if len(k) == 0:
            print(keyframes)
            print(k)
        if k[0] > time:
            keyframe = keyframes[i - 1]
            endKeyframe = keyframes[i]
            break
        else:
            continue
    if keyframe is None:    
        keyframe = keyframes[-1]
    if keyframe[2] is None:
        return keyframe[1]
    if time == keyframe[3]:
        return keyframe[1]
    t = (time - keyframe[3]) / (endKeyframe[0] - keyframe[3])
    t = applyEasing(keyframe[2], t)
    if type(keyframe[1]) is tuple:
        return tuple(keyframe[1][i] + (endKeyframe[1][i] - keyframe[1][i]) * t for i in range(len(keyframe[1])))
    return keyframe[1] + (endKeyframe[1] - keyframe[1]) * t

def keyframeValueAt(keyframes, time):
    if len(keyframes) == 2 and type(keyframes[1]) is list:
        x = k(keyframes[0], time)
        y = k(keyframes[1], time)
        return (x, y)
    else:
        return k(keyframes, time)

# keyframes (time, value, easing, actualStarttime)
def calculatePositionKeyframes(self):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type.startswith("M")]
    if len(applicableEvents) == 0:
        # the initial position is used only if there are no move commands
        keyframes.append((float("-inf"), self.coordinates, None))
        return keyframes
    # the interaction between incompatible commands (Move with MoveX or MoveY, or Scale with ScaleVec)
    # is undefined and i'm not dealing with that. maybe i'll check how lazer does it if i find time
    moveCompatible = True if applicableEvents[0].type == "M" else False
    applicableEvents = [event for event in applicableEvents if event.type == "M"] if moveCompatible else [[event for event in applicableEvents if event.type == "MX"], [event for event in applicableEvents if event.type == "MY"]]
    # two sets of keyframes are used for MX and MY commands
    if moveCompatible:
        i = -2
        for event in applicableEvents:
            i += 2
            appendEndtime = True if event.endtime > event.starttime else False
            if i == 0:
                # the starting event overrides the sprite's initial position
                keyframes.append((float("-inf"), (event.params[0], event.params[1]), None))
                keyframes.append((event.starttime, (event.params[0], event.params[1]) if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                if appendEndtime:
                    keyframes.append((event.endtime, (event.params[2], event.params[3]), None))
                    i += 1
                continue
            if keyframes[i - 1][0] >= event.starttime:
                # the first event overrides subsequent overlapping events,
                # but their interpolation still starts from their respective start times
                keyframes.append((keyframes[i - 1][0], (event.params[0], event.params[1]) if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                i -= 1
            else:
                keyframes.append((event.starttime, (event.params[0], event.params[1]) if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            if appendEndtime:
                keyframes.append((event.endtime, (event.params[2], event.params[3]), None))
            else:
                i -= 1
    else:
        keyframes = [[], []]
        if len(applicableEvents[0]) == 0:
            keyframes[0].append((float("-inf"), self.coordinates[0], None))
        else:
            i = -2
            for event in applicableEvents[0]:
                i += 2
                appendEndtime = True if event.endtime > event.starttime else False
                if i == 0:
                    keyframes[0].append((float("-inf"), event.params[0], None))
                    keyframes[0].append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                    if appendEndtime:
                        keyframes[0].append((event.endtime, event.params[1], None))
                        i += 1
                    continue
                if keyframes[0][i - 1][0] >= event.starttime:
                    keyframes[0].append((keyframes[0][i - 1][0], event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                    i -= 1
                else:
                    keyframes[0].append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                if appendEndtime:
                    keyframes[0].append((event.endtime, event.params[1], None))
                else:
                    i -= 1
        if len(applicableEvents[1]) == 0:
            keyframes[1].append((float("-inf"), self.coordinates[1], None))
        else:
            i = -2
            for event in applicableEvents[1]:
                i += 2
                appendEndtime = True if event.endtime > event.starttime else False
                if i == 0:
                    keyframes[1].append((float("-inf"), event.params[0], None))
                    keyframes[1].append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                    if appendEndtime:
                        keyframes[1].append((event.endtime, event.params[1], None))
                        i += 1
                    continue
                if keyframes[1][i - 1][0] >= event.starttime:
                    keyframes[1].append((keyframes[1][i - 1][0], event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                    i -= 1
                else:
                    keyframes[1].append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
                if appendEndtime:
                    keyframes[1].append((event.endtime, event.params[1], None))
                else:
                    i -= 1
    return keyframes

def calculateRotationKeyframes(self):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type == "R"]
    if len(applicableEvents) == 0:
        # the default rotation is 0
        keyframes.append((float("-inf"), 0, None))
        return keyframes
    i = -2
    for event in applicableEvents:
        i += 2
        appendEndtime = True if event.endtime > event.starttime else False
        if i == 0:
            keyframes.append((float("-inf"), event.params[0], None))
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            if appendEndtime:
                keyframes.append((event.endtime, event.params[1], None))
                i += 1
            continue
        if keyframes[i - 1][0] >= event.starttime:
            keyframes.append((keyframes[i - 1][0], event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            i -= 1
        else:
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
        if appendEndtime:
            keyframes.append((event.endtime, event.params[1], None))
        else:
            i -= 1
    return keyframes

def calculateScaleKeyframes(self):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type == "S" or event.type == "V"]
    if len(applicableEvents) == 0:
        # the default scale is (1, 1)
        keyframes.append((float("-inf"), (1, 1), None))
        return keyframes
    scaleCompatible = True if applicableEvents[0].type == "S" else False
    applicableEvents = [event for event in applicableEvents if ((event.type == "S") if scaleCompatible else event.type == "V")]
    i = -2
    startScale = lambda e : (e.params[0], e.params[0]) if scaleCompatible else (e.params[0], e.params[1])
    endScale = lambda e : (e.params[1], e.params[1]) if scaleCompatible else (e.params[2], e.params[3])
    for event in applicableEvents:
        i += 2
        appendEndtime = True if event.endtime > event.starttime else False
        if i == 0:
            keyframes.append((float("-inf"), startScale(event), None))
            keyframes.append((event.starttime, startScale(event) if appendEndtime else startScale(event), event.easing if appendEndtime else None, event.starttime))
            if appendEndtime:
                keyframes.append((event.endtime, endScale(event), None))
                i += 1
            continue
        if keyframes[i - 1][0] >= event.starttime:
            keyframes.append((keyframes[i - 1][0], startScale(event) if appendEndtime else startScale(event), event.easing if appendEndtime else None, event.starttime))
            i -= 1
        else:
            keyframes.append((event.starttime, startScale(event) if appendEndtime else startScale(event), event.easing if appendEndtime else None, event.starttime))
        if appendEndtime:
            keyframes.append((event.endtime, endScale(event), None))
        else:
            i -= 1
    return keyframes

def calculateColorKeyframes(self):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type == "C"]
    if len(applicableEvents) == 0:
        # the default color is (255, 255, 255)
        keyframes.append((float("-inf"), (255, 255, 255), None))
        return keyframes
    i = -2
    for event in applicableEvents:
        i += 2
        appendEndtime = True if event.endtime > event.starttime else False
        if i == 0:
            keyframes.append((float("-inf"), event.params[0], None))
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            if appendEndtime:
                keyframes.append((event.endtime, event.params[1], None))
                i += 1
            continue
        if keyframes[i - 1][0] >= event.starttime:
            keyframes.append((keyframes[i - 1][0], event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            i -= 1
        else:
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
        if appendEndtime:
            keyframes.append((event.endtime, event.params[1], None))
        else:
            i -= 1
    return keyframes

def calculateOpacityKeyframes(self):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type == "F"]
    if len(applicableEvents) == 0:
        # the default opacity is 1
        keyframes.append((float("-inf"), 1, None))
        return keyframes
    i = -2
    for event in applicableEvents:
        i += 2
        appendEndtime = True if event.endtime > event.starttime else False
        if i == 0:
            keyframes.append((float("-inf"), event.params[0], None))
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            if appendEndtime:
                keyframes.append((event.endtime, event.params[1], None))
                i += 1
            continue
        if keyframes[i - 1][0] >= event.starttime:
            keyframes.append((keyframes[i - 1][0], event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
            i -= 1
        else:
            keyframes.append((event.starttime, event.params[0] if appendEndtime else event.params[1], event.easing if appendEndtime else None, event.starttime))
        if appendEndtime:
            keyframes.append((event.endtime, event.params[1], None))
        else:
            i -= 1
    return keyframes

def calculateEffectKeyframes(self, effect):
    keyframes = []
    applicableEvents = [event for event in self.expandedEvents if event.type == "P" and event.params == effect]
    if len(applicableEvents) == 0:
        # effects are deactivated by default
        keyframes.append((float("-inf"), False, None))
        return keyframes
    i = -2
    for event in applicableEvents:
        i += 2
        appendEndtime = True if event.starttime > event.endtime else False
        if i == 0:
            keyframes.append((float("-inf"), True, None))
            if appendEndtime:
                keyframes.append((event.starttime, False, None))
            else:
                i -= 1
            continue
        if keyframes[i - 1][0] >= event.starttime:
            keyframes.append((keyframes[i - 1][0], True, None))
            i -= 1
        else:
            keyframes.append((event.starttime, True, None))
        if appendEndtime:
            keyframes.append((event.endtime, False, None))
        else:
            i -= 1
    return keyframes

i = 0
class Storyboard:
    def __init__(self, directory, objects, frameSize):
        print(f"Initialising {len(objects)} object{s(objects)}")
        for o in objects:
            o.init()
            #print(o)
        print("Initialised objects")
        self.objects = objects
        self.sprites = []
        self.animations = []
        self.samples = []
        self.spritesAndAnimations = []
        self.beatmapInfo = None
        backgrounds = []
        for o in self.objects:
            if isinstance(o, Sprite):
                self.sprites.append(o)
                self.spritesAndAnimations.append(o)
            if isinstance(o, Animation):
                self.animations.append(o)
                self.spritesAndAnimations.append(o)
            if isinstance(o, Sample):
                self.samples.append(o)
            if isinstance(o, Background):
                backgrounds.append(o)
            if isinstance(o, BeatmapInfo):
                self.beatmapInfo = o.beatmapInfo
        

        # cache and calculate background position beforehand
        self.frameSize = frameSize
        self.background = backgrounds[0] if len(backgrounds) > 0 else None
        self.directory = directory
        backgroundImage = cv.resize(np.array(Image.open(os.path.join(self.directory, self.background.filepath)).convert("RGBA")), (int(frameSize[0] * self.frameSize[1] / float(self.frameSize[1])), self.frameSize[1])) if self.background else None
        self.frameScale = self.frameSize[0] / 854.0
        self.xOffset = int((self.frameSize[0] - self.frameSize[1] * 4 / 3.0) * 0.5) # to centre the image regardless of aspect ratio
        print(f"Background: {None if self.background is None else self.background.filepath}")

        framePosition = (self.xOffset + int(self.background.coordinates[0] * self.frameScale), int(self.background.coordinates[1] * self.frameScale))
        w = backgroundImage.shape[1]
        h = backgroundImage.shape[0]
        y1, y2 = framePosition[1], framePosition[1] + h
        x1, x2 = framePosition[0], framePosition[0] + w
        sy1, sy2 = 0 if y1 >= 0 else -y1, h if y2 <= self.frameSize[1] else h - (y2 - self.frameSize[1])
        sx1, sx2 = 0 if x1 >= 0 else -x1, w if x2 <= self.frameSize[0] else w - (x2 - self.frameSize[0])
        y1 = np.clip(y1, 0, self.frameSize[1])
        y2 = np.clip(y2, 0, self.frameSize[1])
        x1 = np.clip(x1, 0, self.frameSize[0])
        x2 = np.clip(x2, 0, self.frameSize[0])
        imageAlpha = backgroundImage[sy1:sy2, sx1:sx2, 3] / 255.0
        dummyImage = np.array(Image.new("RGB", self.frameSize))
        dummyImageAlpha = 1.0 - imageAlpha[sy1:sy2, sx1:sx2]
        for c in range(3):
            dummyImage[y1:y2, x1:x2, 2 if c == 0 else 0 if c == 2 else 1] = imageAlpha * dummyImage[sy1:sy2, sx1:sx2, c] + dummyImageAlpha * dummyImage[y1:y2, x1:x2, c]
        self.backgroundImage = dummyImage
        self.blankImage = np.array(Image.new("RGB", self.frameSize))

        # cache sprites
        spriteImages = dict()
        for sprite in self.sprites:
            if not sprite.filepath in spriteImages:
                spriteImages[sprite.filepath] = np.array(Image.open(os.path.join(directory, sprite.filepath)).convert("RGBA"))
        self.spriteImages = spriteImages

        animationImages = dict()
        for animation in self.animations:
            if not animation.filepath in animationImages:
                base = ".".join(animation.filepath.split(".")[:-1])
                ext = "." + animation.filepath.split(".")[-1]
                animationImages[animation.filepath] = [np.array(Image.open(os.path.join(directory, base + str(i) + ext)).convert("RGBA")) for i in range(animation.framecount)]
        self.animationImages = animationImages
    
    def generateAudio(self, outFile):
        command = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error"]
        command.append("-i")
        command.append(f"\"{os.path.join(self.directory, self.beatmapInfo['AudioFilename'].strip())}\"")
        delays = [int(self.beatmapInfo["AudioLeadIn"]) if "AudioLeadIn" in self.beatmapInfo else 0]
        for sample in self.samples:
            command.append("-i")
            command.append(f"\"{os.path.join(self.directory, sample.filepath)}\"")
            delays.append(int(sample.starttime))
        command.append("-filter_complex")
        command.append("\"" +
            ";".join([f"[{i}:a]adelay=delays={delays[i]}:all=1[d{i}]" for i in range(len(delays))])
            + ";" + ";".join([f"%samix=inputs={len(delays)}[a]" % "".join([f"[d{i}]" for i in range(len(delays))])])
        + "\"")
        command.append("-map")
        command.append("[a]")
        command.append(outFile)
        #print(" ".join(command))
        sp.Popen(" ".join(command))
    
    def drawFrame(self, time):
        global i
        i += 1
        global t
        global t2
        t2 = pt()
        t = pt()
        frameImage = None

        if self.background is not None and self.background.filepath not in self.spriteImages:
            # TODO: implement video
            frameImage = copy(self.backgroundImage)
        else:
            frameImage = copy(self.blankImage)

        #print(f" Active objects: {len([o for o in self.spritesAndAnimations if o.activetime is not None and o.activetime[0] <= time and o.activetime[1] > time])}           ", end="\r")
        # loop over all active objects
        for o in [o for o in self.spritesAndAnimations if o.activetime is not None and o.activetime[0] <= time and o.activetime[1] > time]:
            # check if sprite should be drawn
            alpha = o.opacityAt(time)
            if alpha == 0:
                continue
            scaleX, scaleY = o.scaleAt(time)
            if scaleX == 0 or scaleY == 0:
                continue

            image = copy(self.spriteImages[o.filepath]) if isinstance(o, Sprite) else copy(self.animationImages[o.filepath][o.frameIndexAt(time)])
            scale = (abs(scaleX) * self.frameScale, abs(scaleY) * self.frameScale)
            newSize = (int(image.shape[1] * scale[0]), int(image.shape[0] * scale[1]))

            # check if resulting sprite is less than a pixel (sort of)
            if newSize[0] <= 0 or newSize[1] <= 0:
                continue

            width1 = newSize[0]
            height1 = newSize[1]
            
            # calculate rotation
            rotation = o.rotationAt(time)
            shouldRotate = True if rotation != 0 else False
            width2, height2 = (newSize[0], newSize[1])
            rotMatrix = None
            if shouldRotate:
                (w, h) = newSize
                (cx, cy) = (w // 2, h // 2)
                rotMatrix = cv.getRotationMatrix2D((cx, cy), -rotation * 180 / math.pi, 1.0)
                cos = np.abs(rotMatrix[0, 0])
                sin = np.abs(rotMatrix[0, 1])
                width2 = int((h * sin) + (w * cos))
                height2 = int((h * cos) + (w * sin))
                rotMatrix[0, 2] += (width2 / 2) - cx
                rotMatrix[1, 2] += (height2 / 2) - cy

            # figure out where the origin point went
            dx = 0.5 * (width2 - width1)
            dy = 0.5 * (height2 - height1)
            midx = width2 * 0.5
            midy = height2 * 0.5 
            origin = (dx, dy) if o.origin == "TopLeft" else (midx, dy) if o.origin == "TopCentre" else (width2 - dx, dy) if o.origin == "TopRight" else (dx, midy) if o.origin == "CentreLeft" else (midx, midy) if o.origin == "Centre" else (width2 - dx, midy) if o.origin == "CentreRight" else (dx, height2 - dy) if o.origin == "BottomLeft" else (midx, height2 - dy) if o.origin == "BottomCentre" else (width2 - dx, height2 - dy) if o.origin == "BottomRight" else None
            if shouldRotate:
                origin = (origin[0] - midx, origin[1] - midy)
                origin = (
                    origin[0] * math.cos(rotation) - origin[1] * math.sin(rotation),
                    origin[1] * math.cos(rotation) + origin[0] * math.sin(rotation)
                )
                origin = (origin[0] + midx, origin[1] + midy)

            position = o.positionAt(time)
            framePosition = (
                int(position[0] * self.frameScale - origin[0]) + self.xOffset,
                int(position[1] * self.frameScale - origin[1])
            )
            y1, y2 = framePosition[1], framePosition[1] + height2
            x1, x2 = framePosition[0], framePosition[0] + width2
            sy1, sy2 = 0 if y1 >= 0 else -y1, height2 if y2 <= self.frameSize[1] else height2 - (y2 - self.frameSize[1])
            sx1, sx2 = 0 if x1 >= 0 else -x1, width2 if x2 <= self.frameSize[0] else width2 - (x2 - self.frameSize[0])
            y1 = np.clip(y1, 0, self.frameSize[1])
            y2 = np.clip(y2, 0, self.frameSize[1])
            x1 = np.clip(x1, 0, self.frameSize[0])
            x2 = np.clip(x2, 0, self.frameSize[0])
            
            # check if in bounds
            if not (y2 - y1 > 0 and x2 - x1 > 0):
                continue

            # apply flips
            flipH = o.effectAt(time, "H") or scaleX < 0
            flipV = o.effectAt(time, "V") or scaleY < 0
            flip = -1 if flipH and flipV else 0 if flipV else 1 if flipH else None
            if flip is not None:
                image = cv.flip(image, flip)

            # apply color and fade
            color = o.colorAt(time)
            for c in range(3):
                if color[c] != 255:
                    np.multiply(image[:, :, c], color[c] / 255.0, casting="unsafe", out=image[:, :, c])
            if alpha != 1:
                np.multiply(image[:, :, 3], alpha, casting="unsafe", out=image[:, :, 3])
            
            # apply scale
            image = cv.resize(image, newSize)

            # apply rotation
            if shouldRotate:
                image = cv.warpAffine(image, rotMatrix, (width2, height2))
            
            # put image on frame
            imageAlpha = image[sy1:sy2, sx1:sx2, 3] / 255.0
            if o.effectAt(time, "A"):
                for c in range(3):
                    frameImage[y1:y2, x1:x2, c] = np.clip(imageAlpha * image[sy1:sy2, sx1:sx2, c] + frameImage[y1:y2, x1:x2, c], 0, 255)
            else:
                frameAlpha = 1.0 - imageAlpha
                for c in range(3):
                    frameImage[y1:y2, x1:x2, c] = imageAlpha * image[sy1:sy2, sx1:sx2, c] + frameAlpha * frameImage[y1:y2, x1:x2, c]
        return frameImage

class Sprite:
    def __init__(self, layer, origin, filepath, coordinates):
        self.layer = layer
        self.origin = origin
        self.filepath = filepath
        self.coordinates = coordinates
        self.events = []
        self.loops = []
        self.triggers = []

    def __repr__(self):
        return "{" + f"\"layer\": \"{self.layer}\", \"origin\": \"{self.origin}\", \"filepath\": \"{self.filepath}\", \"coordinates\": \"{self.coordinates}\", \"events\": {[repr(event) for event in self.events]}, \"loops\": {[repr(loop) for loop in self.loops]}, \"triggers\": {[repr(trigger) for trigger in self.triggers]}".replace("\\", "").replace("'", "") + "}"
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)

    def init(self):
        for loop in self.loops:
            loop.init()
        for trigger in self.triggers:
            trigger.init()

        # TODO: consider triggers
        expandedEvents = self.events
        for loop in self.loops:
            for event in loop.expandedEvents:
                expandedEvents.append(event)
        self.expandedEvents = sorted(expandedEvents, key=lambda event : event.starttime)

        self.activetime = calculateActivetime(self)

        self.positionKeyframes = calculatePositionKeyframes(self)
        self.rotationKeyframes = calculateRotationKeyframes(self)
        self.scaleKeyframes = calculateScaleKeyframes(self)
        self.colorKeyframes = calculateColorKeyframes(self)
        self.opacityKeyframes = calculateOpacityKeyframes(self)
        self.flipHKeyframes = calculateEffectKeyframes(self, "H")
        self.flipVKeyframes = calculateEffectKeyframes(self, "V")
        self.additiveKeyframes = calculateEffectKeyframes(self, "A")
    
    def positionAt(self, time):
        return keyframeValueAt(self.positionKeyframes, time)

    def rotationAt(self, time):
        return keyframeValueAt(self.rotationKeyframes, time)

    def scaleAt(self, time):
        return keyframeValueAt(self.scaleKeyframes, time)

    def colorAt(self, time):
        return keyframeValueAt(self.colorKeyframes, time)

    def opacityAt(self, time):
        return keyframeValueAt(self.opacityKeyframes, time)

    def effectAt(self, time, effect):
        return keyframeValueAt(self.flipHKeyframes if effect == "H" else self.flipVKeyframes if effect == "V" else self.additiveKeyframes if effect == "A" else None, time)

    def addEvent(self, event):
        self.events.append(event)

    def addLoop(self, loop):
        self.loops.append(loop)

    def addTrigger(self, trigger):
        self.triggers.append(trigger)

class Animation:
    def __init__(self, layer, origin, filepath, coordinates, framecount, framedelay, looptype):
        self.layer = layer
        self.origin = origin
        self.filepath = filepath
        self.coordinates = coordinates
        self.framecount = framecount
        self.framedelay = framedelay
        self.looptype = looptype
        self.events = []
        self.loops = []
        self.triggers = []
    
    def __repr__(self):
        return "{" + f"\"layer\": \"{self.layer}\", \"origin\": \"{self.origin}\", \"filepath\": \"{self.filepath}\", \"coordinates\": \"{self.coordinates}\", \"framecount\": {self.framecount}, \"framedelay\": {self.framedelay}, \"looptype\": \"{self.looptype}\", \"events\": {[repr(event) for event in self.events]}, \"loops\": {[repr(loop) for loop in self.loops]}, \"triggers\": {[repr(trigger) for trigger in self.triggers]}".replace("\\", "").replace("'", "") + "}"

    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)

    def init(self):
        for loop in self.loops:
            loop.init()
        for trigger in self.triggers:
            trigger.init()

        # TODO: consider triggers
        expandedEvents = self.events
        for loop in self.loops:
            for event in loop.expandedEvents:
                expandedEvents.append(event)
        self.expandedEvents = sorted(expandedEvents, key=lambda event : event.starttime)

        self.activetime = calculateActivetime(self)
        self.calculateFrameIndex = calculateFrameIndex

        self.positionKeyframes = calculatePositionKeyframes(self)
        self.rotationKeyframes = calculateRotationKeyframes(self)
        self.scaleKeyframes = calculateScaleKeyframes(self)
        self.colorKeyframes = calculateColorKeyframes(self)
        self.opacityKeyframes = calculateOpacityKeyframes(self)
        self.flipHKeyframes = calculateEffectKeyframes(self, "H")
        self.flipVKeyframes = calculateEffectKeyframes(self, "V")
        self.additiveKeyframes = calculateEffectKeyframes(self, "A")
        
    def positionAt(self, time):
        return keyframeValueAt(self.positionKeyframes, time)

    def rotationAt(self, time):
        return keyframeValueAt(self.rotationKeyframes, time)

    def scaleAt(self, time):
        return keyframeValueAt(self.scaleKeyframes, time)

    def colorAt(self, time):
        return keyframeValueAt(self.colorKeyframes, time)

    def opacityAt(self, time):
        return keyframeValueAt(self.opacityKeyframes, time)

    def effectAt(self, time, effect):
        return keyframeValueAt(self.flipHKeyframes if effect == "H" else self.flipVKeyframes if effect == "V" else self.additiveKeyframes if effect == "A" else None, time)

    def frameIndexAt(self, time):
        if hasattr(self, "calculateFrameIndex"):
            frameIndex = self.calculateFrameIndex(self, time)
            if frameIndex >= self.framecount or frameIndex < 0:
                print(self)
                print(frameIndex)
                raise
            return frameIndex
        else:
            raise

    def addEvent(self, event):
        self.events.append(event)
    
    def addLoop(self, loop):
        self.loops.append(loop)

    def addTrigger(self, trigger):
        self.triggers.append(trigger)

class Sample:
    def __init__(self, starttime, layer, filepath, volume):
        self.starttime = starttime
        self.layer = layer
        self.filepath = filepath
        self.volume = volume

    def __repr__(self):
        return ("{" + f"\"time\": {self.time}, \"layer\": \"{self.layer}\", \"filepath\": \"{self.filepath}\", \"volume\": {self.volume}" + "}").replace("\\", "").replace("'", "")
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)

    
    def init(self):
        pass

class Background:
    def __init__(self, filepath, coordinates):
        self.filepath = filepath
        self.coordinates = coordinates

    def __repr__(self):
        return ("{" + f"\"filepath\": \"{self.filepath}\", \"coordinates\": {self.coordinates}" + "}").replace("\\", "").replace("'", "")
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)
    
    def init(self):
        pass

class BeatmapInfo:
    def __init__(self, beatmapInfo):
        self.beatmapInfo = beatmapInfo

    def __repr__(self):
        return ("{" + f"\"beatmapInfo\": \"{self.beatmapInfo}\"" + "}").replace("\\", "").replace("'", "")
        
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)
    
    def init(self):
        pass

class Event:
    def __init__(self, _type, easing, starttime, endtime, params):
        self.type = _type
        self.easing = easing
        self.starttime = starttime
        self.endtime = endtime
        self.params = params

    def __repr__(self):
        return ("{" + f"\"type\": \"{self.type}\", \"easing\": {self.easing}, \"starttime\": {self.starttime}, \"endtime\": {self.endtime}, \"params\": \"{self.params}\"" + "}").replace("\\", "").replace("'", "")
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)


    def valueAt(self, time):
        def interpolate(self, value1, value2, time):
            if self.endtime == self.starttime:
                return value2
            t = (time - self.starttime) / (self.endtime - self.starttime)
            t = applyEasing(self.easing, t)
            return value1 + (value2 - value1) * t
        if self.type == "F" or self.type == "S" or self.type == "R" or self.type == "MX" or self.type == "MY":
            return interpolate(self, self.params[0], self.params[1], time)
        elif self.type == "M" or self.type == "V":
            return (
                interpolate(self, self.params[0], self.params[2], time),
                interpolate(self, self.params[1], self.params[3], time)
            )
        elif self.type == "C":
            return (
                int(interpolate(self, self.params[0][0], self.params[1][0], time)),
                int(interpolate(self, self.params[0][1], self.params[1][1], time)),
                int(interpolate(self, self.params[0][2], self.params[1][2], time))
            )
        elif self.type == "P":
            return True

class Loop:
    def __init__(self, starttime, loopcount):
        self.starttime = starttime
        self.loopcount = loopcount
        self.events = []
        self.expandedEvents = []
    
    def __repr__(self):
        return ("{" + f"\"starttime\": {self.starttime}, \"loopcount\": {self.loopcount}, \"events\": {[repr(event) for event in self.events]}, \"expandedEvents\": {[repr(event) for event in self.expandedEvents]}" + "}").replace("\\", "").replace("'", "")
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)


    def init(self):
        if self.loopcount <= 0:
            # today i learned that loops behave like this
            self.loopcount = 1
        self.looplength = self.events[-1].endtime
        for i in range(self.loopcount):
            for event in self.events:
                newEvent = copy(event)
                newEvent.starttime = self.starttime + event.starttime + self.looplength * i
                newEvent.endtime = self.starttime + event.endtime + self.looplength * i
                self.expandedEvents.append(newEvent)
        self.endtime = self.starttime + (self.events[-1].endtime) * self.loopcount

    def addEvent(self, event):
        self.events.append(event)

class Trigger:
    def __init__(self, triggername, starttime, endtime, groupnumber):
        self.triggername = triggername
        self.starttime = starttime
        self.endtime = endtime
        self.groupnumber = groupnumber
        self.events = []
    
    def __repr__(self):
        return ("{" + f"\"triggername\": \"{self.triggername}\", \"starttime\": {self.starttime}, \"endtime\": {self.endtime}, \"groupnumber\": {self.groupnumber}, \"events\": {[repr(event) for event in self.events]}" + "}").replace("\\", "").replace("'", "")
    
    def __str__(self):
        return json.dumps(json.loads(repr(self)), indent=4)


    def init(self):
        # TODO
        pass

    def addEvent(self, event):
        self.events.append(event)

totalLineNumber = 0
def ParseStoryboard(directory, osbfile, diff=None, inDiff=False):
    variables = []
    objects = []
    with open(os.path.join(directory, osbfile), "r", encoding="utf-8") as file:
        section = ""
        currentObject = None
        currentLoop = None
        currentTrigger = None
        inLoop = False
        inTrigger = False
        lineNumber = 0
        info = dict()
        while True:
            lineNumber += 1
            global totalLineNumber
            totalLineNumber += 1
            line = file.readline()
            if not line:
                break
            #print(line, end="")
             
            if line.startswith("//"):
                continue
            
            # determine start of new section
            if "[Events]" in line:
                if currentObject is not None:
                    if currentLoop is not None:
                        currentObject.addLoop(currentLoop)
                    if currentTrigger is not None:
                        currentObject.addTrigger(currentTrigger)
                    objects.append(currentObject)
                section = "Events"
                continue
            elif "[Variables]" in line:
                if currentObject is not None:
                    if currentLoop is not None:
                        currentObject.addLoop(currentLoop)
                    if currentTrigger is not None:
                        currentObject.addTrigger(currentTrigger)
                    objects.append(currentObject)
                section = "Variables"
                continue
            elif "[General]" in line or "[Metadata]" in line:
                if currentObject is not None:
                    if currentLoop is not None:
                        currentObject.addLoop(currentLoop)
                    if currentTrigger is not None:
                        currentObject.addTrigger(currentTrigger)
                    objects.append(currentObject)
                section = "BeatmapInfo"
                continue
            elif line.strip().startswith("["):
                if currentObject is not None:
                    if currentLoop is not None:
                        currentObject.addLoop(currentLoop)
                    if currentTrigger is not None:
                        currentObject.addTrigger(currentTrigger)
                    objects.append(currentObject)
                section = ""
                continue

            # parse beatmap info
            if section == "BeatmapInfo":
                key = line.split(":")[0].strip()
                value = ":".join(line.split(":")[1:]).strip()
                if key and value:
                    info[key] = value

            # parse variables
            if section == "Variables":
                values = line.split("=")
                if len(values) == 2:
                    if values[1].endswith("\n"):
                        values[1] = values[1][:-1]
                    variables.append((values[0], values[1]))
            
            # parse events
            if section == "Events":
                depth = 0
                while line[depth].startswith(" "):
                    depth += 1
                
                line = applyVariables(line.strip(), variables)
                split = line.split(",")
                
                if inLoop and depth < 2:
                    inLoop = False
                    if currentLoop is None:
                        print(f"ParseError: No loop when there is supposed to be one.\n{osbfile}\nLine {line},\n{currentObject}")
                        raise
                    if len(currentLoop.events) == 0 and currentLoop.loopcount != 0:
                        print(f"ParseError: Empty loop.\n{osbfile}\nLine {lineNumber},\n{currentObject}")
                        raise
                    currentObject.addLoop(currentLoop)

                if inTrigger and depth < 2:
                    inTrigger = False
                    if currentTrigger is None:
                        print(f"ParseError: No trigger when there is supposed to be one.\n{osbfile}\nLine {line},\n{currentObject}")
                        raise
                    currentObject.addTrigger(currentTrigger)

                # new objects
                if split[0] == "Sprite":
                    if currentObject is not None:
                        objects.append(currentObject)
                    layer = split[1]
                    origin = split[2]
                    filepath = removeQuotes(split[3])
                    coordinates = (float(split[4]), float(split[5]))
                    currentObject = Sprite(layer, origin, filepath, coordinates)
                elif split[0] == "Animation":
                    if currentObject is not None:
                        objects.append(currentObject)
                    layer = split[1]
                    origin = split[2]
                    filepath = removeQuotes(split[3])
                    coordinates = (float(split[4]), float(split[5]))
                    framecount = int(split[6])
                    framedelay = float(split[7])
                    looptype = split[8]
                    currentObject = Animation(layer, origin, filepath, coordinates, framecount, framedelay, looptype)
                elif split[0] == "Sample":
                    time = float(split[1])
                    layer = split[2]
                    filepath = removeQuotes(split[3])
                    volume = float(split[4])
                    objects.append(Sample(time, layer, filepath, volume))
                elif split[0] == "0" and split[1] == "0":
                    coordinates = (int(split[3]) + 320 if split[3] else 320, int(split[4]) + 240 if split[4] else 240)
                    objects.append(Background(removeQuotes(split[2]), coordinates))

                # loops and triggers
                elif split[0] == "T":
                    if inLoop:
                        print(f"ParseError: Trigger detected inside loop.\n{osbfile}\nLine {lineNumber}, {line}")
                        raise
                    triggername = split[1]
                    starttime = float(split[2])
                    loopcount = int(split[3])
                    groupnumber = int(split[4]) if len(split) > 4 else 0
                    currentTrigger = Trigger(triggername, starttime, loopcount, groupnumber)
                    inTrigger = True
                elif split[0] == "L":
                    if inTrigger:
                        print(f"ParseError: Loop detected inside trigger.\n{osbfile}\nLine {lineNumber}, {line}")
                        raise
                    starttime = float(split[1])
                    loopcount = int(split[2])
                    currentLoop = Loop(starttime, loopcount)
                    inLoop = True

                # events
                elif split[0] == "F":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startopacity = float(split[4])
                    endopacity = float(split[5]) if len(split) > 5 else startopacity
                    params = (startopacity, endopacity)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "S":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startscale = float(split[4])
                    endscale = float(split[5]) if len(split) > 5 else startscale
                    params = (startscale, endscale)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "V":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startx = float(split[4])
                    starty = float(split[5])
                    endx = float(split[6]) if len(split) > 6 else startx
                    endy = float(split[7]) if len(split) > 7 else starty
                    params = (startx, starty, endx, endy)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "R":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startangle = float(split[4])
                    endangle = float(split[5]) if len(split) > 5 else startangle
                    params = (startangle, endangle)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "M":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startx = float(split[4])
                    starty = float(split[5])
                    endx = float(split[6]) if len(split) > 6 else startx
                    endy = float(split[7]) if len(split) > 7 else starty
                    params = (startx, starty, endx, endy)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "MX":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    startx = float(split[4])
                    endx = float(split[5]) if len(split) > 5 else startx
                    params = (startx, endx)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "MY":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    starty = float(split[4])
                    endy = float(split[5]) if len(split) > 5 else starty
                    params = (starty, endy)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "C":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    color1 = (int(split[4]), int(split[5]), int(split[6]))
                    color2 = (
                        int(split[7]) if len(split) > 7 else color1[0],
                        int(split[8]) if len(split) > 8 else color1[1],
                        int(split[9]) if len(split) > 8 else color1[2]
                    )
                    params = (color1, color2)
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
                elif split[0] == "P":
                    _type = split[0]
                    easing = int(split[1])
                    starttime = float(split[2])
                    endtime = float(split[3]) if split[3] else starttime
                    effect = split[4]
                    params = effect
                    event = Event(_type, easing, starttime, endtime, params)
                    if inTrigger:
                        currentTrigger.addEvent(event)
                    elif inLoop:
                        currentLoop.addEvent(event)
                    else:
                        currentObject.addEvent(event)
        if inLoop:
            inLoop = False
            if currentLoop is None:
                print(f"ParseError: No loop when there is supposed to be one.\n{osbfile}\nLine {lineNumber},\n{currentObject}")
                raise
            if len(currentLoop.events) == 0:
                print(f"ParseError: Empty loop.\n{osbfile}\nLine {lineNumber},\n{currentObject}")
                raise
            currentObject.addLoop(currentLoop)

        if inTrigger:
            inTrigger = False
            if currentTrigger is None:
                print(f"ParseError: No trigger when there is supposed to be one.\n{osbfile}\nLine {lineNumber},\n{currentObject}")
                raise
            currentObject.addTrigger(currentTrigger)
        if currentObject is not None:
            objects.append(currentObject)
        objects.append(BeatmapInfo(info))
        if not inDiff:
            print(f"Parsed {totalLineNumber} line{s(lineNumber)}")
    if diff:
        osu = ParseStoryboard(directory, diff, inDiff=False)
        for o in osu:
            objects.append(o)
    sprites = [o for o in objects if isinstance(o, Sprite)]
    animations = [o for o in objects if isinstance(o, Animation)]
    samples = [o for o in objects if isinstance(o, Sample)]
    if inDiff:
        print(f"Discovered {len(sprites)} sprite{s(sprites)}, {len(animations)} animation{s(animations)} and {len(samples)} sample{s(samples)}")
    return objects

def s(s):
    if type(s) is float:
        return ""
    if type(s) is int:
        return "" if s == 1 else "s"
    return "" if len(s) == 1 else "s"

def removeQuotes(string):
    return string[1:-1] if string.startswith("\"") and string.endswith("\"") else string

def applyVariables(line, variables):
    if not "$" in line:
        return line
    for entry in variables:
        line = line.replace(entry[0], entry[1])
    return line
        
def calculateFrameIndex(self, time):
    if self.looptype == "LoopForever":
        return int(((time - self.activetime[0]) / self.framedelay) % (self.framecount))
    elif self.looptype == "LoopOnce":
        return int(((time - self.activetime[0]) / self.framedelay)) if time - self.activetime[0] < self.framecount * self.framedelay else self.framecount - 1


def calculateActivetime(self):
    return (
        min(
            min([event.starttime for event in self.events]) if len(self.events) > 0 else float("+inf"),
            min([loop.starttime for loop in self.loops if len(loop.events) > 0]) if len([loop for loop in self.loops if len(loop.events) > 0]) > 0 else float("+inf")
        ),
        max(
            max([event.endtime for event in self.events]) if len(self.events) > 0 else float("-inf"),
            max([loop.endtime for loop in self.loops if len(loop.events) > 0]) if len([loop for loop in self.loops if len(loop.events) > 0]) > 0 else float("-inf")
        )
    ) if len(self.events) > 0 or (len([0 for loop in self.loops if len(loop.events) > 0]) > 0) else None

# get osb file
files = os.listdir(sys.argv[1])
osbfile = None
for file in files:
    if file.endswith(".osb"):
        osbfile = file
        break
if osbfile is None:
    print("No .osb file found")
    raise
print(f"Parsing .osb file: {osbfile}")
diff = sys.argv[2] if len(sys.argv) > 2 else None
if diff:
    print(f"Parsing .osu file: {diff}")

frameW = 1920
frameH = 1080
fps = 30
outputFile = "video.mp4"

sb = Storyboard(sys.argv[1], ParseStoryboard(sys.argv[1], osbfile, diff, inDiff=True), (frameW, frameH))
print("Storyboard initialised")

# TODO: implement video background, improve performance
# TODO: think very hard about triggers
starttime = int(sys.argv[3])
duration = int(sys.argv[4])

'''
testframe = sb.drawFrame(int(sys.argv[3]))
testframe = Image.fromarray(testframe)
testframe.show()
'''

frameCount = int(round(fps / (1000.0 / float(duration))))
with Bar("Writing video...", max=frameCount, suffix="%(percent).1f%% - %(eta)ds") as bar:
    fourcc = cv.VideoWriter_fourcc(*"mp4v")
    writer = cv.VideoWriter("export.mp4", fourcc, fps, (frameW, frameH))
    for i in range(frameCount):
        frame = sb.drawFrame(starttime + i * 1000.0 / fps)
        frame = np.flip(frame, axis=2)
        writer.write(frame)
        bar.next()
    writer.release()
print("Generating audio...")
sb.generateAudio("export.mp3")
print("Merging audio and video...")
sp.Popen(f"ffmpeg -y -loglevel error -i export.mp4 -ss {starttime}ms -to {starttime + duration}ms -accurate_seek  -i export.mp3 {outputFile}")
print("Done")
