#pragma once
#include <string>

namespace sb {

    // https://osu.ppy.sh/wiki/en/osu%21_File_Formats/Osu_%28file_format%29#hitsounds
    // https://osu.ppy.sh/wiki/en/Storyboard_Scripting/Compound_Commands
    class HitSound
    {
    public:
        HitSound() {}
        HitSound(int normalSet, int additionSet, int additionFlag, int index)
            :
            normalSet(normalSet),
            additionSet(additionSet),
            additionFlag(additionFlag),
            index(index)
        {}
        HitSound(const std::string& triggerType)
        {
            size_t p = 8;
            if (triggerType.find("All", p) == p) { p += 3; normalSet = -1; }
            else if (triggerType.find("Normal", p) == p) { p += 6; normalSet = 1; }
            else if (triggerType.find("Soft", p) == p) { p += 4; normalSet = 2; }
            else if (triggerType.find("Drum", p) == p) { p += 4; normalSet = 3; }
            else normalSet = -1;

            if (triggerType.find("All", p) == p) { p += 3; additionSet = -1; }
            else if (triggerType.find("Normal", p) == p) { p += 6; additionSet = 1; }
            else if (triggerType.find("Soft", p) == p) { p += 4; additionSet = 2; }
            else if (triggerType.find("Drum", p) == p) { p += 4; additionSet = 3; }
            else additionSet = -1;

            if (triggerType.find("Whistle", p) == p) { p += 7; additionFlag = 2; }
            else if (triggerType.find("Finish", p) == p) { p += 6; additionFlag = 4; }
            else if (triggerType.find("Clap", p) == p) { p += 4; additionFlag = 8; }
            else additionFlag = -1;

            index = triggerType.size() > p ? std::stoi(triggerType.substr(p)) : -1;
        }
        bool operator==(const HitSound& other) const
        {
            return (other.normalSet == -1 ? true : normalSet == other.normalSet)
                && (other.additionSet == -1 ? true : additionSet == other.additionSet)
                && (other.additionFlag == -1 ? true :
                    ((additionFlag & 2) >> 1 && (other.additionFlag & 2) >> 1)
                    || ((additionFlag & 4) >> 2 && (other.additionFlag & 4) >> 2)
                    || ((additionFlag & 8) >> 3 && (other.additionFlag & 8) >> 3))
                && (other.index == -1 ? true : index == other.index);
        }
        static bool IsHitSound(const std::string& triggerType)
        {
            return triggerType.find("HitSound") == 0;
        }

    private:
        char normalSet = 0; // 0 - no sample set, 1 - normal, 2 - soft, 3 - drum
        char additionSet = 0; // 0 - no sample set, 1 - normal, 2 - soft, 3 - drum
        char additionFlag = 0; // bitflag: 0 - normal, 1 - whistle, 2 - finish, 3 - clap
        char index = 0;
    };

}
