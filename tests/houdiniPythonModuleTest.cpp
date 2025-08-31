int foo(GU_Detail *gdp, float tau, UT_Vector3D *to) {
    ntv::nvec3<float> toV(to->x(),to->y(),to->z());
    ntv::nvec3<float> zeeV(0,0,1);
    ntv::nquat<float> qLook;
    qLook.RotateFromTo(zeeV, toV);
    ntv::nquat<float> q(.1,.2,.3,.4);
    q.normalize();
    q = q.slerp(qLook,tau);
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff)
    {
        UT_Vector3 pos = gdp->getPos3(ptoff);
        ntv::nvec3<float> posV(pos.x(),pos.y(),pos.z());
        posV = q*posV;
        //pos.y() += 1;
        pos.assign(posV[0],posV[1],posV[2]);
        gdp->setPos3(ptoff, pos);
    }
    return gdp->getNumPoints();
}
