/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <warp.h>
#include <vector.h>
#include <math.h>

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    Point2f res = Point2f();
    float r = sqrt(sample.x());
    float theta = 2*M_PI*sample.y();
    res.x() = r*cos(theta);
    res.y() = r*sin(theta);
    return res;
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    float res = sqrt(p.x()*p.x() + p.y()*p.y());
    float c = 1/M_PI;
    return (res>1) ? 0 : c;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    Vector3f res = Vector3f();
    float phi = 2.f*M_PI*sample.x();
    float theta = 1.f*acos(sample.y());
    res.x() = sin(theta)*cos(phi);
    res.y() = sin(theta)*sin(phi);
    res.z() = cos(theta);
    return res;
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    float c = 1.f/(2.f*M_PI);
    return (v.z()<0) ? 0 : c;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Vector3f res = Vector3f();
    float phi = 2.f*M_PI*sample.x();
    float theta = 1.f*acos(sqrt(1.f-sample.y()));
    res.x() = sin(theta)*cos(phi);
    res.y() = sin(theta)*sin(phi);
    res.z() = cos(theta);
    return res;
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    float c = v.z()/M_PI;
    return (v.z()<0) ? 0 : c;
}
