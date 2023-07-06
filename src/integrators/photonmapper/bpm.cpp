#include <mitsuba/core/bitmap.h> // ?
#include <mitsuba/core/plugin.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/gatherproc.h>
#include <mitsuba/render/renderqueue.h>

#ifdef MTS_OPENMP
#include <omp.h>
#endif

MTS_NAMESPACE_BEGIN

class BPMIntegrator : public Integrator {
public:
    BPMIntegrator(const Properties& props)
        : Integrator(props)
    {
    }
    template <bool FromLight>
    struct PathVertex {
        Vector3d mHitpoint, mThroughput;
        uint mPathLength;
        const BSDF* mBSDF;
        bool mFromLight = FromLight;
        Float dVM, dVCM; // MIS quantities

        const Vector3d& position() const { return mHitpoint; }
    };

    typedef PathVertex<true> LightVertex;
    typedef PathVertex<false> CameraVertex;

    bool render(Scene* scene, RenderQueue* queue,
        const RenderJob* job, int sceneResID, int sensorResID, int unused)
    {
        return false;
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
        oss << "BPMIntegrator[" << endl
            << "]";
        return oss.str();
    }
    MTS_DECLARE_CLASS()
private:
    bool m_running;
};

MTS_IMPLEMENT_CLASS_S(BPMIntegrator, false, Integrator)
MTS_EXPORT_PLUGIN(BPMIntegrator, "Bidirectional photon mapper")
MTS_NAMESPACE_END