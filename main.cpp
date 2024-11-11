#include <visp/vpFeaturePoint.h>
#include <ecn_baxter_vs/baxter_arm.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <ecn_common/visp_utils.h>

using namespace std;

int main(int argc, char** argv)
{
  BaxterArm arm(argc, argv, "right");    // defaults to right arm
  //  arm.detect(r, g, b, show, saturation, value); to pick detected color, otherwise default

  vpColVector q_init = arm.init(), q;


  //joint limit values
  vpColVector qmin = arm.jointMin(), qmax = arm.jointMax();

  //q star is in the middle between upper and lower joint limits
  vpColVector qd=0.5*(qmax+qmin);
  //upper bounds for safe joint space
  vpColVector qsp=qmax-arm.rho()*(qmax-qmin);
  //lower bounds for safe joint space
  vpColVector qsm=qmin + arm.rho()*(qmax-qmin);
  //i-th upper bound weight
  int hp;
  //i-th lower bound weight
  int hm;



  // define a simple 2D point feature and its desired value
  vpFeaturePoint p,pd;
  pd.set_xyZ(0,0,1);

  // the error vector for visual features (position in the image plane + area of sphere + rotation of x-axis of camera frame)
  vpColVector e(4);
  //error tilde is the extended error vector containing the error on joint position wrt desired safe joint variables
  vpColVector et(11);//et(18);
  double x, y, area;
  // desired area
  const double area_d = arm.area_d();



  // loop variables
  vpColVector qdot(7);
  vpMatrix L(4, 6), Js(4,7);

  while(arm.ok())
  {
    cout << "-------------" << endl;

    // get point features
    x = arm.x();
    y = arm.y();
    area = arm.area();
    p.set_xyZ(x,y, 1);
    std::cout << "x: " << x << ", y: " << y << ", area: " << area << '\n';

    //get joint positions
    q = arm.jointPosition();

    // update error vector e
    e[0]=x-pd.get_x();
    e[1]=y-pd.get_y();
    e[2]=area-area_d;
    e[3]=arm.cameraPose().getRotationMatrix()[2][0];
    ecn::putAt(et,e,0);
    ecn::putAt(et,q-qd,4);
    //ecn::putAt(et,q-qd,11);
    // update interaction matrix L
    vpMatrix Lx=p.interaction();
    vpMatrix La(1,6);
    vpMatrix Lr(1,6);
    La[0][2]=3*area;
    Lr[0][4]=-arm.cameraPose().getRotationMatrix()[2][2];
    Lr[0][5]=arm.cameraPose().getRotationMatrix()[2][1];
    ecn::putAt(L,Lx,0,0);
    ecn::putAt(L,La,2,0);
    ecn::putAt(L,Lr,3,0);
    vpMatrix H(11,11);
    H.eye();


    // build H matrix (2nd section) using arm.rho()
    for(int i=0; i<7; i++){
        hp=ecn::weight(q[i],qsp[i],qmax[i]);
        hm=ecn::weight(-q[i],-qsm[i],-qmin[i]);
        H[i+4][i+4]=hp+hm;

    }

    // compute feature Jacobian from L and cameraJacobian
    const auto J = arm.cameraJacobian(q);
    Js=L*J;
    vpMatrix Jt(11,7);//Jt(18,7);
    ecn::putAt(Jt,Js,0,0);
    vpMatrix Jq(7,7);
    Jq.eye();
    ecn::putAt(Jt,Jq,4,0);

    //to avoid discontinuities we can use constraint on the first 4 joints
    for(int i=4;i<7;i++){
        et[i]=q[i]-q_init[i];
        H[i+4][i+4]=1;
    }


    // send this command to the robot
    vpMatrix HJt=H*Jt;
    qdot=-arm.lambda()*HJt.pseudoInverse()*H*et;
    arm.setJointVelocity(qdot);

    // display current joint positions and VS error
    arm.plot(e);
  }
}
