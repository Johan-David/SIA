#include "material.h"
#include <math.h>
#include "warp.h"

Ward::Ward(const PropertyList &propList)
    : Diffuse(propList.getColor("diffuse",Color3f(0.2)))
{
    m_reflectivity = propList.getColor("reflectivity",Color3f(0.0));
    m_transmissivness = propList.getColor("transmissivness",Color3f(0.0));
    m_etaA = propList.getFloat("etaA",1);
    m_etaB = propList.getFloat("etaB",1);
    m_specularColor = propList.getColor("specular",Color3f(0.9));
    m_alphaX = propList.getFloat("alphaX",0.2);
    m_alphaY = propList.getFloat("alphaY",0.2);

    std::string texturePath = propList.getString("texture","");
    if(texturePath.size()>0){
        filesystem::path filepath = getFileResolver()->resolve(texturePath);
        loadTextureFromFile(filepath.str());
        setTextureScale(propList.getFloat("scale",1));
        setTextureMode(TextureMode(propList.getInteger("mode",0)));
    }
}

Color3f Ward::brdf(const Vector3f& viewDir, const Vector3f& lightDir, const Normal3f& normal, const Vector2f& uv) const
{
    //throw RTException("Ward::brdf() is not yet implemented!");
    Vector3f o = viewDir;
    Vector3f i = lightDir;
    if(i.dot(normal) <=0)
        return Color3f(0.f);
    Vector3f h = (i+o)/(i+o).norm();
    Vector3f d = Vector3f(0.f,1.f,0.f);
    Vector3f x = d-(d.dot(normal))*normal;
    x.normalize();
    Vector3f y = normal.cross(x);
    y.normalize();
    float expo = -((h.dot(x)/m_alphaX)*(h.dot(x)/m_alphaX) + (h.dot(y)/m_alphaY)*(h.dot(y)/m_alphaY))/(h.dot(normal)*h.dot(normal));
    Color3f fr = (m_specularColor/(4*M_PI*m_alphaX*m_alphaY*sqrt(i.dot(normal)*(o.dot(normal))))) * exp(expo);
    return fr+(m_diffuseColor/M_PI);
}


Color3f Ward::premultBrdf(const Vector3f& lightDir, const Vector3f& r, const Normal3f& normal, const Vector2f texcoord) const {
    Vector3f i = lightDir;
    float u = Eigen::internal::random<float>(0,1);
    float v = Eigen::internal::random<float>(0,1);
    float phiH = atan((m_alphaY/m_alphaX)*tan(2.0*M_PI*v));

    float cos1 = (cos(phiH)*cos(phiH))/(m_alphaX*m_alphaX);
    float sin1 = (sin(phiH)*sin(phiH))/(m_alphaY*m_alphaY);

    float thetaH =  atan(sqrt(-log(u)/(cos1 + sin1)));
    Vector3f d = Vector3f(0.f,1.f,0.f);
    Vector3f x = d-(d.dot(normal))*normal;
    x.normalize();
    Vector3f y = normal.cross(x);
    y.normalize();
    Vector3f h = sin(thetaH)*cos(phiH)*x + sin(thetaH)*sin(phiH)*y + cos(thetaH)*normal;
    h.normalize();

    Color3f w = m_specularColor*h.dot(i)*(h.dot(normal)*h.dot(normal)*h.dot(normal))*sqrt(r.dot(normal)/i.dot(normal));

    return w;
}

std::string Ward::toString() const {
    return tfm::format(
        "Ward [\n" 
        "  diffuse color = %s\n"
        "  specular color = %s\n"
        "  alphaX = %f  alphaY = %f\n"
        "]", m_diffuseColor.toString(),
             m_specularColor.toString(),
             m_alphaX, m_alphaY);
}


Vector3f Ward::is(const Normal3f& normal, const Vector3f& lightDir) const {
    Vector3f i = lightDir;
    Vector3f o;
    Vector3f d;
    float randRay = Eigen::internal::random<float>(0,m_specularColor.mean()+m_diffuseColor.mean());
    if(randRay > m_diffuseColor.mean()){
        float u = Eigen::internal::random<float>(0,1);
        float v = Eigen::internal::random<float>(0,1);
        float phiH = atan((m_alphaY/m_alphaX)*tan(2.0*M_PI*v));

        float cos1 = (cos(phiH)*cos(phiH))/(m_alphaX*m_alphaX);
        float sin1 = (sin(phiH)*sin(phiH))/(m_alphaY*m_alphaY);

        float thetaH =  atan(sqrt(-log(u)/(cos1 + sin1)));

        //gestion du cadran
        if((v>0.25) && (v<=0.5)){
            phiH +=M_PI;
        }else if((v>0.5)&&(v<0.75)){
            phiH -=M_PI;
        }

        d = Vector3f(0.f,1.f,0.f);
        Vector3f x = d-(d.dot(normal))*normal;
        x.normalize();
        Vector3f y = normal.cross(x);
        y.normalize();

        Vector3f h = sin(thetaH)*cos(phiH)*x + sin(thetaH)*sin(phiH) * y + cos(thetaH)*normal;
        o = 2.0*(i.dot(h))*h-i;
    }
    else{
        Vector3f s = normal.unitOrthogonal();
        Vector3f t = normal.cross(s);

        Vector3f d;
        d = Warp::squareToCosineHemisphere(Point2f(Eigen::internal::random<float>(0,1),Eigen::internal::random<float>(0,1)));
        float pdf = Warp::squareToCosineHemispherePdf(d);
        o = d.x() * s + d.y() * t + d.z() * normal;
    }
    o.normalize();
    return o;
}

REGISTER_CLASS(Ward, "ward")
