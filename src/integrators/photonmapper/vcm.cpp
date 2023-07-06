/*
    By leavelet, 2023.07.06

*/

#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/render/gatherproc.h>
#include <mitsuba/render/renderqueue.h>

MTS_NAMESPACE_BEGIN

Vector3d getLookat(const Sensor& sensor)
{
    auto cameraMatrix = sensor->getWorldTransform()->eval(0).getMatrix();
    auto cameraPosition = cameraMatrix * Point3f(0, 0, 0);
    auto cameraLookAt = cameraMatrix * Point3f(0, 0, -1); // TODO:Is it correct?
    return cameraLookAt - cameraPosition;
}

Float getFov(const Sensor& sensor)
{
    auto cameraMatrix = sensor->getWorldTransform()->eval(0).getMatrix();
    auto cameraPosition = cameraMatrix * Point3f(0, 0, 0);
    auto cameraLookAt = cameraMatrix * Point3f(0, 0, -1); // TODO:Is it correct?
    auto cameraUp = cameraMatrix * Point3f(0, 1, 0);
    auto cameraFov = std::atan2(cameraUp.y, cameraLookAt.y) * 2;
    return cameraFov;
}

class BPMIntegrator : public Integrator {
public:
    struct SubPathState {
        Vector3d mOrigin, mDirection, mThroughput;
        uint mPathLength = 30, mIsFiniteLight = 1, mSpecularPath = 1;
        Float dVCM, dVM;
    };

    template <bool tFromLight>
    struct PathVertex {
        Vector3d mHitpoint, mThroughput;
        uint mPathLength;

        BSDF* mBSDF;
        Intersection its;

        bool mFromLight = constexpr(tFromLight);

        const Vector3d& getPosition() const { return mHitpoint; }
    };

    typedef PathVertex<false> CameraVertex;
    typedef PathVertex<true> LightVertex;

    class RangeQuery {
    private:
        const BPMIntegrator& mBPMItg;
        const Vector3d& mCameraPosition;
        const CameraVertex& aCameraVertex;
        const SubPathState& mCameraState;
        Vector3d mContrib;

    public:
        RangeQuery(const BPMIntegrator& aBPMint,
            const Vector3d& aCameraPos, const CameraVertex& aCameraVertex, const SubPathState& aCameraState)
            : mBPMItg(aBPMint)
            , mCameraPosition(aCameraPos)
            , aCameraVertex(aCameraVertex)
            , mCameraState(aCameraState)
        {
        }
        const Vector3d& getContrib() const { return mContrib; }
        const Vector3d& getPosition() const { return mCameraPosition; }

        void Process(const LightVertex& aLightVertex)
        {
            if ((aLightVertex.mPathLength + mCameraState.mPathLength > mVertexCM.mMaxPathLength) || (aLightVertex.mPathLength + mCameraState.mPathLength < mVertexCM.mMinPathLength))
                return;
        }
    };

    BPMIntegrator(const Properties& props)
        : Integrator(props)
    {
        mMinPathLength = 0;
        mMaxPathLength = 2;
        mIterations = 0;
        // TODO!
    }

    BPMIntegrator(Stream* stream, InstanceManager* manager)
        : Integrator(stream, manager)
    {
    }

    void serialize(Stream* stream, InstanceManager* manager) const
    {
        Integrator::serialize(stream, manager);
        Log(EError, "Network rendering is not supported!");
    }

    void cancel()
    {
        m_running = false;
    }

    std::string toString() const
    {
        std::ostringstream oss;
        // TODO!
        return oss.str();
    }

    /////////////////////////////////////////////////////////////
    /////           preprossing                           ///////
    /////////////////////////////////////////////////////////////

    bool preprocess(const Scene* scene, RenderQueue* queue, const RenderJob* job,
        int sceneResID, int sensorResID, int samplerResID)
    {
        mBaseRadius = mRadiusFactor * scene->getBSphere().getRadius();
    }

    /////////////////////////////////////////////////////////////
    /////           rendering                             ///////
    /////////////////////////////////////////////////////////////

    bool render(Scene* scene, RenderQueue* queue,
        const RenderJob* job, int sceneResID, int sensorResID, int unused)
    {
        // TODO!
        ref<Scheduler> sched = Scheduler::getInstance();
        ref<Sensor> sensor = scene->getSensor();
        ref<File> film = sensor->getFilm();
        size_t nCores = sched->getCoreCount();

        Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SSE_STR ") ..",
            film->getCropSize().x, film->getCropSize().y,
            nCores, nCores == 1 ? "core" : "cores");

        Vector2i cropSize = file->getCropSize();
        Point2i cropOffset = file->getCropOffset();

        // m_gatherBlocks.clear TODO init storage
        m_running = true;
        m_totalEmitted = 0; // TODO what for?
        m_totalPhotons = 0; // TODO what for?

        ref<Sampler> sampler = static_cast<Sampler*>(PluginManager::getInstance()->createObject(MTS_CLASS(Sampler), Properties("independent"))); // What for?

        const int pathCount = cropSize.x * cropSize.y;
        mScreenPixelCount = pathCount;
        mLightSubPathCount = pathCount;

        int it = 0;
        while (m_running && (m_maxPasses == -1 || it < m_maxPasses)) {
            // TODO!
            Float radius = mBaseRadius;
            radius /= std::pow(Float(it + 1), 0.5f(1 - mRadiusAlpha));
            radius = std::max(radius, 01e-7f);

            const Float radiusSqr = radius * radius;

            mVmNormalization = 1.f / (radiusSqr * M_PIl * mLightSubPathCount);

            // MIS weights
            onst float etaVCM = (M_PIl * radiusSqr) * mLightSubPathCount;
            mMisVmWeightFactor = Mis(1.f / etaVCM);

            mPathEnds.resize(pathCount);
            memset(&mPathEnds[0], 0, sizeof(int) * pathCount);

            mLightVertices.clear();
            mLightVertices.reserve(pathCount);

            bool needsApertureSample = sensor->needsApertureSample();
            bool needsTimeSample = sensor->needsTimeSample();

            ////////////////////////////////////////////////
            // Generate light subpaths                    //
            ////////////////////////////////////////////////
            for (int x = 0; x < cropSize.x; x++) {
                for (int y = 0; y < cropSize.y; y++) {
                    SubPathState lightState;
                    generateLightSample(lightState);
                    //////////////////////////////////////////
                    // Trace light path                     //
                    //////////////////////////////////////////
                    for (;; ++lightState.mPathLength) {
                        RayDifferential ray(lightState.mOrigin + lightState.mDirection * EPS_RAY, lightState.mDirection, 0);
                        Intersection its;

                        if (!scene->rayIntersect(ray, its))
                            break;
                        const Vector3d hitPoint = lightState.mOrigin + lightState.mDirection * its.t;
                        its.t += EPS_RAY;

                        const BSDF* bsdf = its.getBSDF(ray);
                        BSDFSamplingRecord bRec(its, sampler, ERadiance);
                        Float bsdfPdf;
                        Spectrum bsdfValue = bsdf->sample(bRec, bsdfPdf, sampler->next2D());
                        if (bsdfValue.isZero())
                            break;

                        // Update MIS before storing them at the vertex
                        if (lightState.mPathLength > 1 || lightState.mIsFiniteLight) {
                            lightState.dVCM *= Mis(Sqr(its.t));
                        }

                        lightState.dVCM /= Mis(std::abs(bsdf->getFrame().cosTheta(bRec.wo)));
                        lightState.dVM /= Mis(std::abs(bsdf->getFrame().cosTheta(bRec.wo)));

                        // Store vertex unless BSDF is purely specular
                        if (!(bsdf->getType() & BSDF::EDelta)) {
                            LightVertex vertex;
                            vertex.mPosition = hitPoint;
                            vertex.mHitpoint = lightState.mHitpoint;
                            vertex.mThroughput = lightState.mThroughput;
                            vertex.mBSDF = bsdf;

                            vertex.dVCM = lightState.dVCM;
                            vertex.dVM = lightState.dVM;

                            mLightVertices.push_back(vertex);
                        }

                        // Terminate if the path would become too long after scattering
                        if (lightState.mPathLength + 2 > mMaxPathLength)
                            break;

                        // Update path state
                        if (!SampleScattering(bsdf, hitPoint, lightState, true))
                            break;
                    }

                    // Store path end
                    mPathEnds[y * cropSize.x + x] = mLightVertices.size();
                }
            }

            mHashGrid.Reserve(pathCount);
            mHashGrid.Build(mLightVertices, radius);

            for (int x = 0; x < cropSize.x; x++) {
                for (int y = 0; y < cropSize.y; y++) {
                    if (!m_running)
                        break;
                    /////////////////////////////////////////
                    // Generate camera Sample and path    //
                    /////////////////////////////////////////

                    SubPathState cameraState;

                    Point2i pixel(x, y);
                    const int pixelIndex = pixel.y * cropSize.x + pixel.x;
                    const Point2f apertureSample = needsApertureSample ? sampler->next2D() : Point2f(0f);
                    const Float timeSample = needsTimeSample ? sampler->next1D() : 0.f;
                    const Point2f pixelSample = Point2f(pixel) + sampler->next2D();
                    RayDifferential cameraRay = sensor->sampleRayDifferential(cameraRay, pixelSample, apertureSample, timeSample);

                    const Float cosAtCamera = dot(getLookat(sensor), cameraRay.d);
                    const Float cameraFov = sensor->getXFov();
                    const Float tanHalfAngle = tan(cameraFov * 0.5f * M_PI / 180.f);
                    const Float ImagePlaneEist = cropSize.x / (2.f * tanHalfAngle);
                    const Float imagePointToCameraDist = ImagePlaneEist / cosAtCamera;
                    const Float cameraPdfW = Sqr(imagePointToCameraDist) / cosAtCamera;

                    cameraState.mPathLength = 1;
                    cameraState.mSpecularPath = 1;
                    cameraState.mOrigin = cameraRay.o;
                    cameraState.mDirection = cameraRay.d;
                    cameraState.mThroughput = Vector3d(1.f);
                    cameraState.dVM = 0;
                    cameraState.dVCM = Mis(mLightSubPathCount * cameraPdfW);

                    Vector3d color(0);

                    //////////////////////////////////////////
                    // Trace camera path                   //
                    //////////////////////////////////////////
                    for (;; ++cameraState.mPathLength) {
                        RayDifferential newray(cameraState.mOrigin + cameraState.mDirection * EPS_RAY, cameraState.mDirection, Ray(cameraRay).time); // TODO, time?
                        Intersection its;
                        if (scene->rayIntersect(newray, its)) {
                            if (scene->hasEnvironmentEmitter()) {
                                if (cameraState.mPathLength >= mMinPathLength) {
                                    color += cameraState.mThroughput * scene->evalEnvironmentEmitter(ray);
                                }
                            }
                            break;
                        }
                        const Vector3d hitPoint = newray.o + newray.d * its.t;
                        its.t += EPS_RAY;

                        const BSDF* bsdf = its.getBSDF(newray);
                        BSDFSamplingRecord bRec(its, sampler, ERadiance);
                        Float bsdfPdf;
                        Spectrum bsdfValue = bsdf->sample(bRec, bsdfPdf, sampler->next2D());
                        if (bsdfValue.isZero())
                            break;

                        // Update MIS before storing them at the vertex
                        cameraState.dVCM *= Mis(Sqr(isect.dist));
                        cameraState.dVCM /= Mis(std::abs(Frame::cosTheta(bRec.wo)));
                        cameraState.dVM /= Mis(std::abs(Frame::cosTheta(bRec.wo)));
                    }

                    if (its.emitter) {
                        color += cameraState.mThroughput * its.emitter->eval(its, -cameraState.mDirection);
                    }

                    if (cameraState.mPathLength >= mMaxPathLength) {
                        break;
                    }

                    ////////////////////////////////////////////////////////////////
                    // Vertex merging: Merge with light vertices
                    if (!bsdf->getType() & BSDF::EDelta) {
                        RangeQuery query(*this, hitPoint, bsdf, cameraState);
                        mHashGrid.Process(mLightVertices, query);
                        color += cameraState.mThroughput * mVmNormalization * query.GetContrib();

                    }

                    if (!SampleScattering(bsdf, hitPoint, cameraState))
                        break;
                    mBitmap->setPixel(pixel, color);
                }
            }
            it++;
        }
    }

    void generateCameraPath(const Scene* scene, Sampler* sampler, const SubPathState& state, LightVertex& vertex)
    {
    }

    MTS_DECLARE_CLASS()
private:
    // Mis, balance heuristic
    float Mis(float aPdf) const
    {
        return aPdf;
    }

    void GenerateLightSample(Scene* scene, Sampler* sampler, SubPathState& oLightState)
    {
        ref_vector<Emitter> emitters = scene->getEmitters();
        const emitterCount = emitters.size();
        const Float lightPickProb = 1.f / emitterCount;

        const int lightID = int(sampler->next1D() * emitterCount) % emitterCount;
        const Vector2d rndDirSamples = sampler->next2D();
        const Vector2d rndPosSamples = sampler->next2D();

        const Emitter* emitter = emitters[lightID].get();
        Float emissionPdfW, directPdfA, cosLight;

        DirectionSamplingRecord dRec(rndDirSamples);
        PositionSamplingRecord pRec;
        emitter->samplePosition(pRec, rndPosSamples);
        oLightState.mThroughput = emitter->evalDirection(dRec, pRec);
        oLightState.mOrigin = pRec.p;
        oLightState.mDirection = dRec.d;

        emissionPdfW = emitter->pdfPosition(pRec);
        directPdfA = emitter->pdfDirection(dRec, pRec);
        cosLight = std::abs(dRec.d.z);
        if (emitter->getType != 0)
            cosLight = 1.f;

        emissionPdfW *= lightPickProb;
        directPdfA *= lightPickProb;

        oLightState.mThroughput /= emissionPdfW;
        oLightState.mPathLength = 1;
        oLightState.mIsFiniteLight = !(emitter->getType() & Emitter::EEmitterType::EOnSurface); // not on surface means infinite

        oLightState.dVM = Mis(directPdfA / emissionPdfW);

        if ((emitter->getType() & Emitter::EEmitterType::EDeltaPosition) || (emitter->getType() & Emitter::EEmitterType::EDeltaDirection))
            oLightState.dVM = 0;
        else
            oLightState.dVM = Mis(cosLight / emissionPdfW) * mMisVcWeightFactor;
    }

    struct ComponentProbabilities {
        float diffProb;
        float phongProb;
        float reflProb;
        float refrProb;
    };

    ComponentProbabilities mCProb;

    bool SampleScattering(BSDF* bsdf, Intersection& its, *sampler, Vector3d hitPoint, SubPathState& lightState, bool Light)
    {
        BSDFSamplingRecord bRec(its, sampler->next2D(), ERadiance);
        bRec.typeMask float bsdfDirPdfW, cosThetaOut;
        uint sampledEvent;
        bsdf->sample(bRec, bsdfDirPdfW, sampler->next2D());
        cosThetaOut = bsdf->getFrame().cosTheta(bRec.wo);
        sampledEvent = bRec.eventType;

        float bsdfRevPdfW = bsdfDirPdfW;
        if (sampledEvent & ESingular == 0)
            bsdfRevPdfW = bsdf->pdf(bRec); // TODO: check

        if (sampledEvent & EDeltaReflection || sampledEvent & EDeltaTransmission)
            return false;

        if (sampledEvent & BSDF::EDiffuse) {
            mCProb.diffProb = sampler->next1D() * 200;
        }

        if (sampledEvent & BSDF::EGlossy) {
            mCProb.phongProb = sampler->next1D() * 200;
        }

        if (sampledEvent & BSDF::EReflection) {
            mCProb.reflProb = sampler->next1D() * 200;
        }

        if (sampledEvent & BSDF::ERefraction) {
            mCProb.refrProb = sampler->next1D() * 200;
        }

        const float totalAlbedo = mCProb.diffProb + mCProb.phongProb + mCProb.reflProb + mCProb.refrProb;
        mCProb.diffProb /= totalAlbedo;
        mCProb.phongProb /= totalAlbedo;
        mCProb.reflProb /= totalAlbedo;
        mCProb.refrProb /= totalAlbedo;

        Float mContinuationProb = std::min(1, std::max(mCProb.diffProb, samper->next1D() / 10 + std::max(mCProb.phongProb, std::max(mCProb.reflProb, mCProb.refrProb))));

        if (mContinuationProb < sampler->next1D())
            return false;

        bsdfRevPdfW *= mContinuationProb;
        bsdfDirPdfW *= mContinuationProb;

        if (sampledEvent & BSDF::EDiffuse) {
            lightState.dVCM = 0;
            lightState.dVC = Mis(cosThetaOut);

            lightState.mSpecularPath &= 1;
        } else {
            lightState.dVM = Mis(cosThetaOut / bsdfDirPdfW) * (lightState.dVM * Mis(bsdfRevPdfW) + lightState.dVCM * mMisVcWeightFactor + 1.f);

            lightState.dVCM = Mis(1.f / bsdfDirPdfW);

            lightState.mSpecularPath &= 0;
            lightState.dVM = Mis(cosThetaOut / bsdfDirPdfW) * (lightState.dVM * Mis(bsdfRevPdfW) + lightState.dVCM * mMisVcWeightFactor + 1.f);

            lightState.dVCM = Mis(1.f / bsdfDirPdfW);

            lightState.mSpecularPath &= 0;
        }

        lightState.mOrigin = hitPoint;
        lightState.mDirection = bRec.wo;
        lightState.mThroughput *= bRec.wo * (bsdf->getFrame().cosTheta(bRec.wo) / bsdfDirPdfW);
        return true;
    }

private:
    bool m_running = false;
    const Float mRadiusFactor;
    const Float mRadiusAlpha;
    int mSeed = 1234;
    const Float EPS_RAY = 1e-3f;
    Float mBaseRadius;
    uint mMaxPathLength, mMinPathLength;
    uint m_maxPasses = -1; // TODO, init
    int mIterations, m_totalEmitted = 0, m_totalPhotons = 0;
    Float mScreenPixelCount, mLightSubPathCount;
    Float mMisVcWeightFactor;
    std::vector<int> mPathEnds;
    HashGrid mHashGrid;
    std::vector<LightVertex> mLightVertices; //!< Stored light vertices
    ref<Bitmap> mBitmap;
};

MTS_IMPLEMENT_CLASS_S(BPMIntegrator, false, Integrator)
MTS_EXPORT_PLUGIN(BPMIntegrator, "Bidirectional Photon Mapping integrator");
MTS_NAMESPACE_END