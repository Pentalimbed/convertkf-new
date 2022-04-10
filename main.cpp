#include "headers.h"

#define COMPRESSED

#include <iostream>
#include <string>
#include <vector>
#include <map>

#pragma region Niflib Headers
//////////////////////////////////////////////////////////////////////////
// Niflib Includes
//////////////////////////////////////////////////////////////////////////
#define USE_NIFLIB_TEMPLATE_HELPERS

#include <niflib.h>
#include <nif_io.h>
#include <nif_math.h>
#include "obj/NiObject.h"
#include "obj/NiNode.h"
#include "obj/NiTexturingProperty.h"
#include "obj/NiSourceTexture.h"
#include "obj/NiTriBasedGeom.h"
#include "obj/NiTriBasedGeomData.h"
#include "obj/NiTriShape.h"
#include "obj/NiTriStrips.h"
#include <obj/NiControllerSequence.h>
#include <obj/NiControllerManager.h>
#include <obj/NiInterpolator.h>
#include <obj/NiTransformInterpolator.h>
#include <obj/NiTransformData.h>
#include <obj/NiTransformController.h>
#include <obj/NiTimeController.h>
#include <obj/NiTransformController.h>
#include <obj/NiTextKeyExtraData.h>
#include <obj/NiKeyframeController.h>
#include <obj/NiKeyframeData.h>
#include <obj/NiStringPalette.h>
#include <obj/NiBSplineTransformInterpolator.h>
#include <obj/NiDefaultAVObjectPalette.h>
#include <obj/NiMultiTargetTransformController.h>
#include <obj/NiGeomMorpherController.h>
#include <obj/NiMorphData.h>
#include <obj/NiBSplineCompFloatInterpolator.h>
#include <obj/NiFloatInterpolator.h>
#include <obj/NiFloatData.h>
#include <Key.h>

typedef Niflib::Key<float>              FloatKey;
typedef Niflib::Key<Niflib::Quaternion> QuatKey;
typedef Niflib::Key<Niflib::Vector3>    Vector3Key;
typedef Niflib::Key<string>             StringKey;

using namespace Niflib;

#pragma endregion

#pragma region Havok Headers
//////////////////////////////////////////////////////////////////////////
// Havok Includes
//////////////////////////////////////////////////////////////////////////
#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>
#include <Common/Base/System/Io/IStream/hkIStream.h>

// Scene
#include <Common/SceneData/Scene/hkxScene.h>
#include <Common/Serialize/Util/hkRootLevelContainer.h>
#include <Common/Serialize/Util/hkLoader.h>

// Animation
#include <Animation/Animation/Rig/hkaSkeleton.h>
#include <Animation/Animation/Rig/hkaPose.h>
#include <Animation/Animation/Rig/hkaSkeletonUtils.h>
#include <Animation/Animation/hkaAnimationContainer.h>
#include <Animation/Animation/Mapper/hkaSkeletonMapper.h>
#include <Animation/Animation/Playback/Control/Default/hkaDefaultAnimationControl.h>
#include <Animation/Animation/Playback/hkaAnimatedSkeleton.h>
#include <Animation/Animation/Animation/SplineCompressed/hkaSplineCompressedAnimation.h>
#include <Animation/Animation/Animation/Quantized/hkaQuantizedAnimation.h>
#include <Animation/Animation/Animation/Util/hkaAdditiveAnimationUtility.h>
#include <Animation/Animation/Playback/hkaAnimatedSkeleton.h>

#include <Common/Serialize/Util/hkLoader.h>
#include <Common/Serialize/Util/hkRootLevelContainer.h>
#include <Common/Compat/Deprecated/Packfile/Xml/hkXmlPackfileWriter.h>

// Serialize
#include <Common/Serialize/Util/hkSerializeUtil.h>

#pragma endregion

#define LOG(str) std::cout << std::endl \
                           << str

static void HK_CALL debugReport(const char* msg, void* userContext)
{
    LOG(msg);
}

//////////////////////////////////////////////////////////////////////////
// Enumeration Types
//////////////////////////////////////////////////////////////////////////

enum PosRotScale
{
    prsPos     = 0x1,
    prsRot     = 0x2,
    prsScale   = 0x4,
    prsDefault = prsPos | prsRot | prsScale,
};

//////////////////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////////////////

const unsigned int IntegerInf    = 0x7f7fffff;
const unsigned int IntegerNegInf = 0xff7fffff;
const float        FloatINF      = *(float*)&IntegerInf;
const float        FloatNegINF   = *(float*)&IntegerNegInf;

//////////////////////////////////////////////////////////////////////////
// Helper Funcs
//////////////////////////////////////////////////////////////////////////

namespace
{
static inline Niflib::Vector3 TOVECTOR3(const hkVector4& v)
{
    return Niflib::Vector3(v.getSimdAt(0), v.getSimdAt(1), v.getSimdAt(2));
}

static inline Niflib::Vector4 TOVECTOR4(const hkVector4& v)
{
    return Niflib::Vector4(v.getSimdAt(0), v.getSimdAt(1), v.getSimdAt(2), v.getSimdAt(3));
}

static inline hkVector4 TOVECTOR4(const Niflib::Vector4& v)
{
    return hkVector4(v.x, v.y, v.z, v.w);
}

static inline Niflib::Quaternion TOQUAT(const hkQuaternion& q, bool inverse = false)
{
    Niflib::Quaternion qt(q.m_vec.getSimdAt(3), q.m_vec.getSimdAt(0), q.m_vec.getSimdAt(1), q.m_vec.getSimdAt(2));
    return inverse ? qt.Inverse() : qt;
}

static inline hkQuaternion TOQUAT(const Niflib::Quaternion& q, bool inverse = false)
{
    hkVector4 v(q.x, q.y, q.z, q.w);
    v.normalize4();
    hkQuaternion qt(v.getSimdAt(0), v.getSimdAt(1), v.getSimdAt(2), v.getSimdAt(3));
    if (inverse) qt.setInverse(qt);
    return qt;
}

static inline Niflib::Quaternion TOQUAT(const hkRotation& rot, bool inverse = false)
{
    return TOQUAT(hkQuaternion(rot), inverse);
}
static inline float Average(const Niflib::Vector3& val)
{
    return (val.x + val.y + val.z) / 3.0f;
}

float QuatDot(const Quaternion& q, const Quaternion& p)
{
    return q.w * p.w + q.x * p.x + q.y * p.y + q.z * p.z;
}

const float        MY_FLT_EPSILON = 1e-5f;
static inline bool EQUALS(float a, float b)
{
    return fabs(a - b) < MY_FLT_EPSILON;
}
static inline int COMPARE(float a, float b)
{
    float d = a - b;
    return (fabs(d) < MY_FLT_EPSILON ? 0 : (d > 0 ? 1 : -1));
}

static inline bool EQUALS(const Niflib::Vector3& a, const Niflib::Vector3& b)
{
    return (EQUALS(a.x, b.x) && EQUALS(a.y, b.y) && EQUALS(a.z, b.z));
}

static inline bool EQUALS(const Niflib::Quaternion& a, const Niflib::Quaternion& b)
{
    return EQUALS(a.w, b.w) && EQUALS(a.x, b.x) && EQUALS(a.y, b.y) && EQUALS(a.z, b.z);
}

const float FramesPerSecond = 30.0f;
//const float FramesIncrement = 0.0325f;
const float FramesIncrement = 0.033333f;
} // namespace

static void FillTransforms(hkArray<hkQsTransform>& transforms, int boneIdx, int numTracks, const hkQsTransform& localTransform, PosRotScale prs = prsDefault, int from = 0, int to = -1)
{
    int n = transforms.getSize() / numTracks;
    if (n == 0)
        return;

    if (to == -1 || to >= n) to = n - 1;

    if ((prs & prsDefault) == prsDefault)
    {
        for (int idx = from; idx <= to; ++idx)
        {
            hkQsTransform& transform = transforms[idx * numTracks + boneIdx];
            transform                = localTransform;
        }
    }
    else
    {
        for (int idx = from; idx <= to; ++idx)
        {
            hkQsTransform& transform = transforms[idx * numTracks + boneIdx];
            if ((prs & prsPos) != 0)
                transform.setTranslation(localTransform.getTranslation());
            if ((prs & prsRot) != 0)
                transform.setRotation(localTransform.getRotation());
            if ((prs & prsScale) != 0)
                transform.setScale(localTransform.getScale());
        }
    }
}
static void SetTransformPosition(hkQsTransform& transform, hkVector4& p)
{
    if (p.getSimdAt(0) != FloatNegINF) transform.setTranslation(p);
}
static void SetTransformRotation(hkQsTransform& transform, hkQuaternion& q)
{
    if (q.m_vec.getSimdAt(3) != FloatNegINF) transform.setRotation(q);
}
static void SetTransformScale(hkQsTransform& transform, float s)
{
    if (s != FloatNegINF) transform.setScale(hkVector4(s, s, s));
}
static void PosRotScaleNode(hkQsTransform& transform, hkVector4& p, hkQuaternion& q, float s, PosRotScale prs)
{
    if (prs & prsScale) SetTransformScale(transform, s);
    if (prs & prsRot) SetTransformRotation(transform, q);
    if (prs & prsPos) SetTransformPosition(transform, p);
}
static void SetTransformPositionRange(hkArray<hkQsTransform>& transforms, int numTracks, int boneIdx, float& currentTime, float lastTime, int& frame, Vector3Key& first, Vector3Key& last)
{
    int       n = transforms.getSize() / numTracks;
    hkVector4 p = TOVECTOR4(first.data);
    for (; COMPARE(currentTime, lastTime) <= 0 && frame < n; currentTime += FramesIncrement, ++frame)
    {
        hkQsTransform& transform = transforms[frame * numTracks + boneIdx];
        SetTransformPosition(transform, p);
    }
}
static void SetTransformRotationRange(hkArray<hkQsTransform>& transforms, int numTracks, int boneIdx, float& currentTime, float lastTime, int& frame, QuatKey& first, QuatKey& last)
{
    int          n = transforms.getSize() / numTracks;
    hkQuaternion q = TOQUAT(first.data);
    for (; COMPARE(currentTime, lastTime) <= 0 && frame < n; currentTime += FramesIncrement, ++frame)
    {
        hkQsTransform& transform = transforms[frame * numTracks + boneIdx];
        SetTransformRotation(transform, q);
    }
}
static void SetTransformScaleRange(hkArray<hkQsTransform>& transforms, int numTracks, int boneIdx, float& currentTime, float lastTime, int& frame, FloatKey& first, FloatKey& last)
{
    int n = transforms.getSize() / numTracks;
    for (; COMPARE(currentTime, lastTime) <= 0 && frame < n; currentTime += FramesIncrement, ++frame)
    {
        hkQsTransform& transform = transforms[frame * numTracks + boneIdx];
        SetTransformScale(transform, first.data);
    }
}

float roundf(float x)
{
    float res;
    if (x >= 0.0F)
    {
        res = ceilf(x);
        if (res - x > 0.5F) res -= 1.0F;
    }
    else
    {
        res = ceilf(-x);
        if (res + x > 0.5F) res -= 1.0F;
        res = -res;
    }
    return res;
}

//////////////////////////////////////////////////////////////////////////
// Stuff
//////////////////////////////////////////////////////////////////////////

void exportController(NiControllerSequenceRef       seq,
                      hkRefPtr<hkaAnimationBinding> binding,
                      hkRefPtr<hkaSkeleton>         skeleton)
{
    auto blocks = seq->GetControlledBlocks();
    int  nbones = skeleton->m_bones.getSize();

    // NoRootSiblings ...

    int   numTracks = nbones;
    float duration  = seq->GetStopTime() - seq->GetStartTime();
    int   nframes   = (int)roundf(duration / FramesIncrement);

    hkRefPtr<hkaInterleavedUncompressedAnimation> tempAnim = new hkaInterleavedUncompressedAnimation();
    tempAnim->m_duration                                   = duration;
    tempAnim->m_numberOfTransformTracks                    = numTracks;
    tempAnim->m_numberOfFloatTracks                        = 0; //anim->m_numberOfFloatTracks;
    tempAnim->m_transforms.setSize(numTracks * nframes, hkQsTransform::getIdentity());
    tempAnim->m_floats.setSize(tempAnim->m_numberOfFloatTracks);
    tempAnim->m_annotationTracks.setSize(numTracks);

    hkArray<hkQsTransform>& transforms = tempAnim->m_transforms;

    struct comp
    {
        bool operator()(const std::string& lhs, const std::string& rhs) const
        {
            return stricmp(lhs.c_str(), rhs.c_str()) < 0;
        }
    }; // case insensitive
    std::map<std::string, int, comp> boneMap;
    for (int i = 0; i < nbones; ++i)
    {
        string name   = skeleton->m_bones[i].m_name;
        boneMap[name] = i;
        // + anno
        tempAnim->m_annotationTracks[i].m_trackName = name.c_str();
    }

    for (auto& bone : blocks)
    {
        auto boneitr = boneMap.find(bone.nodeName);
        if (boneitr == boneMap.end())
        {
            LOG("Unknown bone " << bone.nodeName.c_str() << " found in animation. Skipping.");
            continue;
        }

        int           boneIdx        = boneitr->second;
        hkQsTransform localTransform = skeleton->m_referencePose[boneIdx];
        FillTransforms(transforms, boneIdx, nbones, localTransform); // prefill transforms with bindpose

        NiTransformInterpolatorRef interpolator = DynamicCast<NiTransformInterpolator>(bone.interpolator);
        NiTransformDataRef         data         = interpolator->GetData();
        if (data->GetTranslateType() == Niflib::LINEAR_KEY)
        {
            std::vector<Vector3Key> keys = data->GetTranslateKeys();
            int                     n    = keys.size();
            if (n > 0)
            {
                int         frame       = 0;
                float       currentTime = 0.0f;
                Vector3Key *itr = &keys[0], *last = &keys[n - 1];
                SetTransformPositionRange(transforms, nbones, boneIdx, currentTime, (*itr).time, frame, *itr, *itr);
                for (int i = 1; i < n; ++i)
                {
                    Vector3Key* next = &keys[i];
                    SetTransformPositionRange(transforms, nbones, boneIdx, currentTime, (*next).time, frame, *itr, *next);
                    itr = next;
                }
                SetTransformPositionRange(transforms, nbones, boneIdx, currentTime, duration, frame, *last, *last);
            }
        }
        else
        {
            LOG("Missing transform data for " << boneitr->first.c_str());
        }

        if (data->GetRotateType() == Niflib::QUADRATIC_KEY)
        {
            vector<QuatKey> keys = data->GetQuatRotateKeys();
            int             n    = keys.size();
            if (n > 0)
            {
                int      frame       = 0;
                float    currentTime = 0.0f;
                QuatKey *itr = &keys[0], *last = &keys[n - 1];
                SetTransformRotationRange(transforms, nbones, boneIdx, currentTime, itr->time, frame, *itr, *itr);
                for (int i = 1; i < n; ++i)
                {
                    QuatKey* next = &keys[i];
                    SetTransformRotationRange(transforms, nbones, boneIdx, currentTime, next->time, frame, *itr, *next);
                    itr = next;
                }
                SetTransformRotationRange(transforms, nbones, boneIdx, currentTime, duration, frame, *last, *last);
            }
        }
        else
        {
            LOG("Missing rotation data for " << boneitr->first.c_str());
        }

        if (data->GetScaleType() == Niflib::LINEAR_KEY)
        {
            vector<FloatKey> keys = data->GetScaleKeys();
            int              n    = keys.size();
            if (n > 0)
            {
                int       frame       = 0;
                float     currentTime = 0.0f;
                FloatKey *itr = &keys[0], *last = &keys[n - 1];
                SetTransformScaleRange(transforms, nbones, boneIdx, currentTime, itr->time, frame, *itr, *itr);
                for (int i = 1; i < n; ++i)
                {
                    FloatKey* next = &keys[i];
                    SetTransformScaleRange(transforms, nbones, boneIdx, currentTime, next->time, frame, *itr, *next);
                    itr = next;
                }
                SetTransformScaleRange(transforms, nbones, boneIdx, currentTime, duration, frame, *last, *last);
            }
        }
        else
        {
            LOG("Missing scaling data for " << boneitr->first.c_str());
        }
    }

    hkaSkeletonUtils::normalizeRotations(transforms.begin(), transforms.getSize());

    // create the animation with default settings
    {
#ifdef COMPRESSED
        hkaSplineCompressedAnimation::TrackCompressionParams     tparams;
        hkaSplineCompressedAnimation::AnimationCompressionParams aparams;

        tparams.m_rotationTolerance        = 0.001f;
        tparams.m_rotationQuantizationType = hkaSplineCompressedAnimation::TrackCompressionParams::THREECOMP40;

        hkRefPtr<hkaSplineCompressedAnimation> outAnim = new hkaSplineCompressedAnimation(*tempAnim.val(), tparams, aparams);

        binding->m_animation = outAnim;
#else
        binding->m_animation = tempAnim;
#endif
        binding->m_originalSkeletonName = skeleton->m_bones[0].m_name;
    }
}

void convertkf(std::string skelPath, std::string kfPath, std::string outPath)
{
    hkSerializeUtil::SaveOptionBits flags = hkSerializeUtil::SAVE_DEFAULT;
    hkPackfileWriter::Options       packFileOptions;
    packFileOptions.m_layout = hkStructureLayout::MsvcWin32LayoutRules;

    hkResource*  skelResource = NULL;
    hkResource*  animResource = NULL;
    hkaSkeleton* skeleton     = NULL;

    LOG("Loading skeleton!");
    hkSerializeUtil::ErrorDetails loadError;
    skelResource = hkSerializeUtil::load(skelPath.c_str(), &loadError);
    if (!skelResource)
    {
        LOG("Could not load " << skelPath << std::endl
                              << " Error: " << loadError.defaultMessage.cString());
        return;
    }
    else
    {
        const char*            hktypename   = skelResource->getContentsTypeName();
        void*                  contentPtr   = skelResource->getContentsPointer(HK_NULL, HK_NULL);
        hkRootLevelContainer*  scene        = skelResource->getContents<hkRootLevelContainer>();
        hkaAnimationContainer* skelAnimCont = scene->findObject<hkaAnimationContainer>();
        if (!skelAnimCont->m_skeletons.isEmpty())
            skeleton = skelAnimCont->m_skeletons[0];
    }
    if (!skeleton)
    {
        LOG(skelPath << "has no skeleton! Abort!");
        return;
    }

    LOG("Reading animation!");
    auto blocks = DynamicCast<NiControllerSequence>(Niflib::ReadNifList(kfPath.c_str(), NULL));
    if (blocks.size() == 0)
    {
        LOG("Animation file contains no animation bindings. Not exporting.");
        return;
    }
    else if (blocks.size() != 1)
    {
        LOG("Animation file contains more than one animation binding. Not exporting.");
        return;
    }


    auto                            seq = blocks[0];
    hkRootLevelContainer            rootCont;
    hkRefPtr<hkaAnimationContainer> skelAnimCont = new hkaAnimationContainer();
    hkRefPtr<hkaAnimationBinding>   newBinding   = new hkaAnimationBinding();
    skelAnimCont->m_bindings.append(&newBinding, 1);
    rootCont.m_namedVariants.pushBack(hkRootLevelContainer::NamedVariant("Merged Animation Container", skelAnimCont.val(), &skelAnimCont->staticClass()));

    LOG("Converting!");
    exportController(seq, newBinding, skeleton);

    LOG("Exporting!");
    skelAnimCont->m_animations.pushBack(newBinding->m_animation);

    hkOstream stream(outPath.c_str());
    hkVariant root = {&rootCont, &rootCont.staticClass()};
    hkResult  res  = hkSerializeUtil::savePackfile(root.m_object, *root.m_class, stream.getStreamWriter(), packFileOptions, HK_NULL, flags);
    if (res == HK_SUCCESS)
    {
        LOG("HKX file Successfully exported!");
    }
    else
    {
        LOG("Failed to save export file!");
    }

    hkOstream           stream_xml((outPath + ".xml").c_str());
    hkXmlPackfileWriter writer;
    writer.setContents(&rootCont, *root.m_class);
    res = writer.save(stream_xml.getStreamWriter(), packFileOptions);

    if (res == HK_SUCCESS)
    {
        LOG("XML HKX file Successfully exported!");
    }
    else
    {
        LOG("Failed to save export file!");
    }
}

int main(int argc, char* argv[])
{
    hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
    hkBaseSystem::init(memoryRouter, debugReport);

    std::string skelPath;
    std::string kfPath;
    std::string outPath;

    bool skip = false;
    if (true)
    {
        if (argc != 4)
        {
            LOG("Wrong parameters. Example: convertkf skeleton.hkx anim.kf output.hkx");
            skip = true;
        }
        else
        {
            skelPath = argv[1];
            kfPath   = argv[2];
            outPath  = argv[3];
        }
    }
    else
    {
        skelPath = "E:/dev/projects/convertkf-new/hkx/skeleton.hkx";
        kfPath   = "E:/dev/projects/convertkf-new/hkx/test.kf";
        outPath  = "E:/dev/projects/convertkf-new/hkx/test.hkx";
    }

    if (!skip)
        convertkf(skelPath, kfPath, outPath);

    hkBaseSystem::quit();
    hkMemoryInitUtil::quit();
    return 0;
}