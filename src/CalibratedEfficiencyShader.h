#pragma once
#include <string>
namespace spraythickness::opengl {
inline const char* kCalibratedEfficiencyShader=R"GLSL(
uniform bool useCalibratedEfficiencies;
uniform bool efficiencyIncludesGeometry;
uniform int distanceCurveKind,angleCurveKind,distanceCurveCount,angleCurveCount;
uniform float distanceCurveParameters[6],angleCurveParameters[6];
uniform vec2 distanceCurveNodes[128],angleCurveNodes[128];
float calibratedEfficiency(bool isDistance,float x){
    int k=isDistance?distanceCurveKind:angleCurveKind;
    float p[6];for(int i=0;i<6;++i)p[i]=isDistance?distanceCurveParameters[i]:angleCurveParameters[i];
    if(k==0)return 1.;
    if(k==1)return 1./(.001443*x*x-.3073*x+28.31);
    if(k==2){x=clamp(x,0.,180.);return .0002564*x*x+.01058*x+.6358;}
    if(k>=3 && k<=6){float v=0.;for(int i=0;i<k-1;++i)v=v*x+p[i];return v;}
    if(k==7)return p[0]*exp(p[1]*x);
    if(k==8)return p[0]*exp(p[1]*x)+p[2]*exp(p[3]*x);
    if(k==9)return p[0]*pow(max(x,1e-9),p[1]);
    if(k==10)return p[0]*pow(max(x,1e-9),p[1])+p[2];
    if(k==11 || k==12){float s=max(abs(p[2]),1e-9);float v=p[0]*exp(-pow((x-p[1])/s,2.));
        if(k==12){s=max(abs(p[5]),1e-9);v+=p[3]*exp(-pow((x-p[4])/s,2.));}return v;}
    float den=1.;float num=0.;
    if(k==13){num=p[0]*x+p[1];den=x+p[2];}
    if(k==14){num=p[0]*x*x+p[1]*x+p[2];den=x+p[3];}
    if(k==15){num=p[0]*x*x+p[1]*x+p[2];den=x*x+p[3]*x+p[4];}
    if(k>=13 && k<=15)return num/(abs(den)<1e-9?1e-9:den);
    int n=isDistance?distanceCurveCount:angleCurveCount;
    if(n<=0)return 1.;
    vec2 prev=isDistance?distanceCurveNodes[0]:angleCurveNodes[0];
    if(x<=prev.x)return prev.y;
    for(int i=1;i<n;++i){vec2 next=isDistance?distanceCurveNodes[i]:angleCurveNodes[i];
        if(x<=next.x)return mix(prev.y,next.y,(x-prev.x)/max(next.x-prev.x,1e-9));prev=next;}
    return prev.y;
}
)GLSL";
inline std::string addCalibratedEfficiencyShader(std::string source){
    const auto at=source.find("vec3 safeNormalize");source.insert(at,kCalibratedEfficiencyShader);return source;
}
}
