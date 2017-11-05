#include "material.h"
#include <math.h>

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
    float expo = -((h.dot(x)/m_alphaX)*(h.dot(x)/m_alphaX) + (h.dot(y)/m_alphaY)*(h.dot(y)/m_alphaY))/(h.dot(normal)*h.dot(normal));
    Color3f fr = (m_specularColor/(4*M_PI*m_alphaX*m_alphaY*sqrt(i.dot(normal)*(o.dot(normal))))) * exp(expo);
    return fr+(m_diffuseColor/M_PI);
}


Color3f Ward::premultBrdf(const Vector3f& lightDir, const Vector3f& r, const Normal3f& normal, const Vector2f texcoord){
    Vector3f i = lightDir;
    float u = Eigen::internal::random<float>(0,1);
    float v = Eigen::internal::random<float>(0,1);
    float phiH = atan((m_alphaY/m_alphaX)*tan(2.0*M_PI*v));

    float cos1 = (cos(phiH)*cos(phiH))/(m_alphaX*m_alphaX);
    float sin1 = (sin(phiH)*sin(phiH))/(m_alphaY*m_alphaY);

    float thetaH =  atan(sqrt(-log(u)/(cos1+ sin1)));
    Vector3f h = Vector3f(sin(thetaH)*cos(phiH), sin(thetaH)*sin(phiH), cos(thetaH));

    float expo = -(tan(thetaH)*tan(thetaH))*(cos1+sin1);
    Color3f fr = (m_specularColor/(4*M_PI*m_alphaX*m_alphaY*sqrt(i.dot(normal)*(o.dot(normal))))) * exp(expo);
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
    float u = Eigen::internal::random<float>(0,1);
    float v = Eigen::internal::random<float>(0,1);
    float phiH = atan((m_alphaY/m_alphaX)*tan(2.0*M_PI*v));

    float cos1 = (cos(phiH)*cos(phiH))/(m_alphaX*m_alphaX);
    float sin1 = (sin(phiH)*sin(phiH))/(m_alphaY*m_alphaY);

    float thetaH =  atan(sqrt(-log(u)/(cos1+ sin1)));
    Vector3f h = Vector3f(sin(thetaH)*cos(phiH), sin(thetaH)*sin(phiH), cos(thetaH));
    Vector3f o = 2.0*(i.dot(h))*h-i;
    return o;
}

REGISTER_CLASS(Ward, "ward")
