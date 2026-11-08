#pragma once
#include "PaperGaussianHistoryShader.h"

namespace spraythickness::opengl {
// GLSL has no double-precision exp overload. Range reduction keeps the optimizer
// and long thermal accumulations in fp64 without float-quantized finite steps.
inline const char* kCalibrationExp=R"GLSL(
double preciseExp(double x) {
    if(x < -700.0) return 0.0;
    x=min(x,700.0);
    int k=int(round(x*1.4426950408889634074LF));
    double r=(x-double(k)*0.69314718055994528623LF)-double(k)*2.3190468138462995584e-17LF;
    // |r| <= log(2)/2: ten terms keep relative truncation below 3e-13.
    double term=1.0,sum=1.0;
    for(int j=1;j<=10;++j){term*=r/double(j);sum+=term;}
    return ldexp(sum,k);
}
double injection(double x) {
    return x<0.001 ? x*(1.0-x*(0.5-x*(1.0/6.0-x/24.0))) : 1.0-preciseExp(-x);
}
)GLSL";
inline std::string calibrationFieldShader() {
    auto source=makePaperGaussianHistoryShader(false,false);
    source.resize(source.find("void evaluateThicknessVertex()"));
    return source+kCalibrationExp+R"GLSL(
layout(std430,binding=3) buffer FitState { dvec4 fitState[]; };
layout(std430,binding=4) buffer FitOutput { dvec4 fitOutput[]; };
layout(std430,binding=11) readonly buffer FitRates { vec4 cachedRates[]; };
layout(std430,binding=12) readonly buffer FitEvents { dvec4 events[]; };
layout(std430,binding=13) readonly buffer FitDecay { dvec2 decayAndInput[]; };
uniform int vertexBegin,vertexEnd,eventBegin,eventEnd,mode;
uniform bool ratesCached,heatEnabled;
uniform double initialTau,heatAmplitude,heatScale,coolingScale,referenceQ,referenceRate,threshold;
uniform bool continuous;
vec3 rateAt(uint vertex,int pose) {
    if(ratesCached)return cachedRates[(vertex-uint(vertexBegin))*uint(sprayPointCount)+uint(pose)].xyz;
    vec3 ray=surfaceData[vertex*2u].xyz-sprayData[pose*5].xyz;
    float axial=dot(ray,sprayData[pose*5+1].xyz),radius=length(ray);
    if(axial<=0.001 || radius<=0.0)return vec3(0.0);
    float cosine=dot(safeNormalize(surfaceData[vertex*2u+1u].xyz,vec3(0,0,1)),-ray/radius);
    if(cosine<=0.0)return vec3(0.0);
    float pattern=patternValue(dot(ray,sprayData[pose*5+2].xyz),dot(ray,sprayData[pose*5+3].xyz),axial);
    float angle=0.0,rate=incidentRate(radius,cosine,pattern,angle)*1000.0;
    return vec3(rate*densityThicknessScale(radius,angle),rate,pattern);
}
void main() {
    uint i=gl_GlobalInvocationID.x+uint(vertexBegin);if(i>=uint(vertexEnd))return;
    dvec4 state=eventBegin==0?dvec4(initialTau,0,0,0):fitState[i];
    dvec4 accumulated=eventBegin==0?dvec4(0):fitOutput[i];
    for(int j=eventBegin;j<eventEnd;++j){
        dvec4 event=events[j];double dt=event.y,time=event.z;
        vec3 rates=rateAt(i,int(event.x));
        // Preserve the CPU fit convention: skipped zero-flux events still cool
        // existing history before the next visible contribution.
        if(heatEnabled){
            state.x*=decayAndInput[j].x;
            if(rates.x>0.0 || mode==1){
                if(continuous)state.x+=decayAndInput[j].y
                    *clamp(double(rates.y)/referenceRate,0.0,1.0);
                else if(mode!=1 && double(rates.z)>=threshold)state.x+=dt;
            }
        }
        state.y=time;
        if(mode!=1 && rates.x<=0.0)continue;
        double base=double(rates.x)*dt;
        double difference=heatEnabled?preciseExp(-state.x/heatScale)-referenceQ:0.0;
        double positive=continuous?difference:max(0.0,difference),negative=continuous?0.0:min(0.0,difference);
        double factor=heatEnabled?1.0+heatAmplitude*(continuous || heatAmplitude>=0.0?positive:negative):1.0;
        accumulated.x+=mode==1?dt*factor:base*factor;
        accumulated.y+=base;accumulated.z+=base*positive;accumulated.w+=base*negative;
    }
    fitState[i]=state;fitOutput[i]=accumulated;
}
)GLSL";
}
inline const char* kCalibrationStatistics=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Values { dvec4 values[]; };
layout(std430,binding=1) readonly buffer Observed { double observed[]; };
layout(std430,binding=2) buffer Partial { double partial[]; };
uniform int count,groups;
shared double lanes[128];
void main(){
    uint column=gl_WorkGroupID.y,lane=gl_LocalInvocationID.x;
    double value=0.0;
    for(uint i=gl_GlobalInvocationID.x;i<uint(count);i+=uint(groups)*128u){
        dvec4 p=values[i];double o=observed[i],r=p.x-o,b=o-p.y,v=0.0;
        if(column==0u)v=r*r;else if(column==1u)v=p.x*o;else if(column==2u)v=p.x*p.x;
        else if(column==3u)v=p.x;else if(column==4u)v=b*b;
        else if(column==5u)v=p.z*b;else if(column==6u)v=p.z*p.z;
        else if(column==7u)v=p.w*b;else v=p.w*p.w;
        value=column==3u?max(value,v):value+v;
    }
    lanes[lane]=value;barrier();
    for(uint stride=64u;stride>0u;stride>>=1u){
        if(lane<stride)lanes[lane]=column==3u?max(lanes[lane],lanes[lane+stride]):lanes[lane]+lanes[lane+stride];barrier();}
    if(lane==0u)partial[column*uint(groups)+gl_WorkGroupID.x]=lanes[0];
}
)GLSL";
inline const char* kCalibrationReduce=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Partial { double partial[]; };
layout(std430,binding=1) buffer Result { double result[]; };
uniform int groups,maxColumn;
shared double lanes[128];
void main(){
    uint column=gl_WorkGroupID.x,lane=gl_LocalInvocationID.x;
    double value=lane<uint(groups)?partial[column*uint(groups)+lane]:0.0;
    lanes[lane]=value;barrier();
    for(uint stride=64u;stride>0u;stride>>=1u){
        if(lane<stride)lanes[lane]=int(column)==maxColumn?max(lanes[lane],lanes[lane+stride]):lanes[lane]+lanes[lane+stride];barrier();}
    if(lane==0u)result[column]=lanes[0];
}
)GLSL";
inline const char* kCalibrationGather=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Values { dvec4 values[]; };
layout(std430,binding=1) readonly buffer Indices { int indices[]; };
layout(std430,binding=2) buffer Result { double result[]; };
uniform int count;uniform bool subset;
void main(){uint i=gl_GlobalInvocationID.x;if(i<uint(count))result[i]=values[subset?indices[i]:int(i)].x;}
)GLSL";
inline std::string probabilityValuesShader(){return std::string(R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Points { dvec4 points[]; };
layout(std430,binding=1) readonly buffer Params { double params[]; };
layout(std430,binding=2) buffer Values { double values[]; };
uniform int count,components,dimensions;
uniform double scaleU,scaleV;
)GLSL")+kCalibrationExp+R"GLSL(
void main(){
    uint i=gl_GlobalInvocationID.x;if(i>=uint(count))return;
    double maximum=0.0;int offset=0;
    for(int k=0;k<components;++k){if(k>0)maximum=max(maximum,params[offset+4]);offset+=k>0?5:4;}
    double total=0.0;offset=0;uint row=i*uint(dimensions+1);
    for(int k=0;k<components;++k){
        double inverseU=preciseExp(-params[offset+2])/scaleU,inverseV=preciseExp(-params[offset+3])/scaleV;
        double a=(points[i].x-params[offset]*scaleU)*inverseU,b=(points[i].y-params[offset+1]*scaleV)*inverseV;
        double t=preciseExp((k>0?params[offset+4]:0.0)-maximum-0.5*(a*a+b*b));total+=t;
        values[row+uint(offset+1)]=t*a*inverseU*scaleU;values[row+uint(offset+2)]=t*b*inverseV*scaleV;
        values[row+uint(offset+3)]=t*a*a;values[row+uint(offset+4)]=t*b*b;
        if(k>0)values[row+uint(offset+5)]=t;
        offset+=k>0?5:4;
    }
    values[row]=total;
}
)GLSL";}
inline const char* kProbabilityReduction=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Points { dvec4 points[]; };
layout(std430,binding=1) readonly buffer Values { double values[]; };
layout(std430,binding=2) readonly buffer Normalization { double normalization[]; };
layout(std430,binding=3) buffer Partial { double partial[]; };
uniform int count,dimensions,groups,phase,fitThickness;uniform double inverseLossScale;
shared double lanes[128];
void main(){
    uint column=gl_WorkGroupID.y,lane=gl_LocalInvocationID.x;double sum=0.0;
    double z=phase==0?1.0:max(normalization[0],1e-300LF);
    double amplitude=0.0;
    if(fitThickness!=0 && phase!=0)amplitude=max(0.0,normalization[0]/max(normalization[1],1e-300LF));
    for(uint i=gl_GlobalInvocationID.x;i<uint(count);i+=uint(groups)*128u){
        uint row=i*uint(dimensions+1);
        if(phase==0){
            if(fitThickness!=0){double q=points[i].z*values[row];
                sum+=column==0u?q*points[i].w:q*q;}
            else sum+=points[i].z*values[row+column];
        }
        else {
            if(fitThickness!=0){
                double r=amplitude*points[i].z*values[row]-points[i].w;
                if(column==0u)sum+=r*r*inverseLossScale;
                else if(column>1u)sum+=2.0*amplitude*r*inverseLossScale*points[i].z*values[row+column-1u];
            }else{
                double probability=values[row]/z,r=probability-points[i].w;
                if(column==0u)sum+=r*r*inverseLossScale;
                else if(column==1u)sum+=2.0*r*inverseLossScale*probability;
                else sum+=2.0*r*inverseLossScale*values[row+column-1u];
            }
        }
    }
    lanes[lane]=sum;barrier();
    for(uint stride=64u;stride>0u;stride>>=1u){if(lane<stride)lanes[lane]+=lanes[lane+stride];barrier();}
    if(lane==0u)partial[column*uint(groups)+gl_WorkGroupID.x]=lanes[0];
}
)GLSL";
inline const char* kProbabilityGradient=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Normalization { double normalization[]; };
layout(std430,binding=1) buffer Result { double result[]; };
uniform int dimensions;
void main(){uint i=gl_GlobalInvocationID.x;if(i<uint(dimensions))
    result[i+2u]=(result[i+2u]-result[1]*normalization[i+1u])/max(normalization[0],1e-300LF);}
)GLSL";
}
