#include <cisstCommon/cmnGetChar.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawOpenDynamicsEngine/mtsODEWorld.h>
#include <sawOpenSceneGraph/mtsOSGMono.h>
#include <sawOpenDynamicsEngine/mtsODEBarrettHand.h>

class BHMotion : public mtsTaskPeriodic {

private:
  
  mtsFunctionRead  GetPositions;
  mtsFunctionWrite SetPositions;

  vctDynamicVector<double> q;

public:

  BHMotion() : mtsTaskPeriodic( "BHMotion", 0.001, true ){

    q.SetSize(4);
    q.SetAll(0.0);

    mtsInterfaceRequired* input = AddInterfaceRequired( "Input" );
    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );

    input->AddFunction( "GetPositionJoint", GetPositions );
    output->AddFunction( "SetPositionJoint", SetPositions );

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();
    
    prmPositionJointGet qin;
    GetPositions( qin );
    
    prmPositionJointSet qout;
    qout.SetSize( 4 );
    qout.Goal() = q;
    SetPositions( qout );
    
    for( size_t i=0; i<q.size(); i++ ) q[i] += 0.001;

  }
  
  void Cleanup(){}

};

int main(){

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  osg::ref_ptr< mtsODEWorld > world = NULL;
  world = new mtsODEWorld( "world", 0.0001, vctFixedSizeVector<double,3>(0.0) );
  taskManager->AddComponent( world.get() );
  
  // Create a camera
  int x = 0, y = 0;
  int width = 640, height = 480;
  double Znear = 0.1, Zfar = 10.0;
  mtsOSGMono* camera;
  camera = new mtsOSGMono( "camera",
                           world,
                           x, y,
                           width, height,
                           55.0, ((double)width)/((double)height),
                           Znear, Zfar );
  taskManager->AddComponent( camera );

  std::string path( CISST_SOURCE_ROOT"/etc/cisstRobot/BH/" );
  vctFrame4x4<double> Rtw0;
  Rtw0[2][3] = 0.1;

  mtsODEBarrettHand* BH;
  BH = new mtsODEBarrettHand( "BH",
			      0.001,
			      OSA_CPU1,
			      20,
			      path + "l0.obj",
			      path + "l1.obj",
			      path + "l2.obj",
			      path + "l3.obj",
			      world,
			      Rtw0,
			      path + "f1f2.rob",
			      path + "f3.rob" );
  taskManager->AddComponent( BH );
  
  BHMotion motion;
  taskManager->AddComponent( &motion );

  taskManager->Connect( motion.GetName(), "Input",  BH->GetName(), "Output" );
  taskManager->Connect( motion.GetName(), "Output", BH->GetName(), "Input" );

  taskManager->CreateAll();
  taskManager->WaitForStateAll( mtsComponentState::READY );

  taskManager->StartAll();
  taskManager->WaitForStateAll( mtsComponentState::ACTIVE );

  cmnGetChar();
  std::cout << "ESC to quit" << std::endl;
  cmnGetChar();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;

}
