#include "integrator.h"
#include "scene.h"
#include "material.h"
#include "warp.h"

class AO : public Integrator
{
public:
    AO(const PropertyList &props) {
        m_sampleCount = props.getInteger("samples", 10);
        m_cosineWeighted = props.getBoolean("cosine", true);
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {

        Hit hit;
        scene->intersect(ray, hit);
        Color3f c = Color3f(0.f);
        if (hit.foundIntersection())
        {
            Normal3f n = hit.normal();
            Point3f pos = ray.at(hit.t());

            for(int i=0; i<m_sampleCount; i++){

                float x = Eigen::internal::random<float>(0,1);
                float y = Eigen::internal::random<float>(0,1);

//                Vector3f d = Warp::squareToUniformHemisphere(Point2f(x, y));
                Vector3f d = Warp::squareToCosineHemisphere(Point2f(x, y));

                Vector3f u;
                if(n.y() == 0 && n.z() == 0){
                    u = n.cross(Vector3f(0,0,1));
                }
                else{
                    u = n.cross(Vector3f(1,0,0));
                }
                u.normalize();

                Vector3f v;
                v = u.cross(n);
                v.normalize();

                Vector3f p = d.x() * u + d.y() * v + d.z() * n;

                Ray shadow_ray(pos + n*Epsilon, p, true);
                Hit shadow_hit;
                scene->intersect(shadow_ray, shadow_hit);

                float pdf;
//                pdf = Warp::squareToUniformHemispherePdf(d);
                pdf = Warp::squareToCosineHemispherePdf(d);

                if(shadow_hit.foundIntersection()){
                        c += Color3f(1.0f)*((p.dot(n) / (p.norm() * n.norm())) / M_PI) / pdf;
                }
              }
              c /= m_sampleCount;
            c = Color3f(1.0f) - c;

            }
        return c;
    }

    std::string toString() const {
      return tfm::format("AO[\n"
                         "  samples = %f\n"
                         "  cosine-weighted = %s]\n",
                         m_sampleCount, 
                         m_cosineWeighted ? "true" : "false");
  }

private:
    int m_sampleCount;
    bool m_cosineWeighted;
};

REGISTER_CLASS(AO, "ao")
