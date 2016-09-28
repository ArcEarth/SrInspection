#include "pch_bcl.h"
#include "CausalityApplication.h"
//#include "Content\CubeScene.h"
//#include "Content\SampleFpsTextRenderer.h"
#include <CommonStates.h>
#include <PrimitiveVisualizer.h>
#include <ppltasks.h>

#include "KinectSensor.h"
#include "OculusRift.h"
#include "LeapMotion.h"
#include "Vicon.h"

#include <tinyxml2.h>
#include "Tests.h"

using namespace Causality;
using namespace std;
namespace sys = std::tr2::sys;
using namespace DirectX;
using namespace DirectX::Scene;

string  g_AppManifest = "App.xml";

//std::wstring sceneFile = L"SelectorScene.xml";

App::App()
{
}

App::~App()
{
	for (auto& pCom : Components)
	{
		UnregisterComponent(pCom.get());
	}
}

bool App::OnStartup(const std::vector<std::string>& args)
{
	using namespace tinyxml2;
	tinyxml2::XMLDocument appDoc(true, Whitespace::COLLAPSE_WHITESPACE);
	auto error = appDoc.LoadFile(g_AppManifest.c_str());
	if (error != XML_SUCCESS)
	{
		string message = "Failed to load or parse file : " + g_AppManifest;
		MessageBoxA(NULL, message.c_str(), "Startup failed", MB_OK);
		return false;
	}

	auto appSettings = appDoc.FirstChildElement("application");
	if (!appSettings)
	{
		MessageBoxA(NULL, "App.xml is not formated correct", "Startup failed", MB_OK);
		return false;
	}

	auto windowSettings = appSettings->FirstChildElement("window");
	auto consoleSettings = appSettings->FirstChildElement("console");

	string assetDir;
	GetParam(appSettings->FirstChildElement("assets"), "path", assetDir);
	string scenestr;
	GetParam(appSettings->FirstChildElement("scenes"), "path", scenestr);

	m_assetsDir = assetDir;
	path sceneFile = scenestr;

	if (m_assetsDir.empty())
		m_assetsDir = sys::current_path();
	else if (m_assetsDir.is_relative())
		m_assetsDir = sys::current_path() / m_assetsDir;

	if (sceneFile.is_relative())
		sceneFile = m_assetsDir / sceneFile;
	if (!sys::exists(sceneFile))
	{
		string message = "Secen file doest exist : " + sceneFile.string();
		MessageBoxA(NULL, message.c_str(), "Startup failed", MB_OK);
		return false;
	}

	string title = "No title";
	GetParam(appSettings, "title", title);

	unsigned width = 1280, height = 720;
	int x, y;
	bool fullscreen = false;

	// Initialize Windows
	if (consoleSettings)
	{
		GetParam(consoleSettings, "width", width);
		GetParam(consoleSettings, "height", height);
		GetParam(consoleSettings, "fullscreen", fullscreen);
		GetParam(consoleSettings, "left", x);
		GetParam(consoleSettings, "top", y);

		pConsole = make_shared<DebugConsole>();
		pConsole->Initialize(title, width, height, fullscreen);
		pConsole->Move(x, y);
	}

	bool runTest = false;
	GetParam(appSettings, "run_test", runTest);
	if (runTest && !TestManager::RunTest())
	{
		std::cout << "[Error] Test Failed! Press any key to exit" << std::endl;
		return false;
	}

	if (windowSettings)
	{
		GetParam(windowSettings, "width", width);
		GetParam(windowSettings, "height", height);
		GetParam(windowSettings, "fullscreen", fullscreen);
		GetParam(windowSettings, "left", x);
		GetParam(windowSettings, "top", y);
	}

	pWindow = make_shared<NativeWindow>();
	if (!pRift)
	{
		pWindow->Initialize(title, width, height, fullscreen);
		pWindow->Move(x, y);
	}
	else
	{
		//auto res = pRift->Resoulution();
		Vector2 res = { 1920, 1080 };
		pWindow->Initialize(std::string(title), (unsigned)res.x, (unsigned)res.y, false);
	}
	//bool useOvr = Devices::OculusRift::Initialize();

	// Initialize DirectX
	pDeviceResources = make_shared<DirectX::DeviceResources>();
	pDeviceResources->SetNativeWindow(pWindow->Handle());
	// Register to be notified if the Device is lost or recreated
	pDeviceResources->RegisterDeviceNotify(this);
	pWindow->SizeChanged += MakeEventHandler(&App::OnResize, this);

	//return;

	pDeviceResources->GetD3DDevice()->AddRef();
	pDeviceResources->GetD3DDeviceContext()->AddRef();
	pDevice.Attach(pDeviceResources->GetD3DDevice());
	pContext.Attach(pDeviceResources->GetD3DDeviceContext());

	Visualizers::g_PrimitiveDrawer.Initialize(pContext.Get());

	// Oculus Rift
	//if (pRift)
	//{
	//	if (!pRift->InitializeGraphics(pWindow->Handle(), pDeviceResources.get()))
	//		pRift = nullptr;
	//}

	SetupDevices(appSettings);
	//auto loadingScene = new Scene;
	//Scenes.emplace_back(loadingScene);
	//loadingScene->SetRenderDeviceAndContext(pDevice, pContext);
	//loadingScene->SetCanvas(pDeviceResources->GetBackBufferRenderTarget());

	Scenes.emplace_back(new Scene);
	auto& scene = Scenes.back();
	scene->SetRenderDeviceAndContext(pDevice.Get(), pContext.Get());
	scene->SetHudRenderDevice(pDeviceResources->GetD2DFactory(), pDeviceResources->GetD2DDeviceContext(), pDeviceResources->GetDWriteFactory());
	scene->SetCanvas(pDeviceResources->GetBackBufferRenderTarget());

	concurrency::task<void> loadScene([sceneFile,&scene]() {
		cout << "Current Directory :" << sys::current_path() << endl;
		cout << "Loading [Scene](" << sceneFile << ") ..." << endl;
		CoInitializeEx(NULL, COINIT::COINIT_APARTMENTTHREADED);
		scene->LoadFromFile(sceneFile.string());
		CoUninitialize();
		cout << "[Scene] Loading Finished!";
	});

	return true;
}

void App::SetupDevices(const ParamArchive* arch)
{
	bool enable = false;
	auto setting = arch->FirstChildElement("vicon");
	GetParam(setting, "enable", enable);
	if (setting && enable)
	{
		pVicon = Devices::IViconClient::Create();
		if (pVicon)
		{
			pVicon->Initialize(setting);
			pVicon->Start();
		}
	}

#if defined(__HAS_LEAP__)
	setting = arch->FirstChildElement("leap");
	GetParam(setting, "enable", enable);
	if (setting && enable)
	{
		pLeap = Devices::LeapSensor::GetForCurrentView();
		pLeap->Initialize(setting);
	}
#endif

#if defined(__HAS_KINECT__)
	setting = arch->FirstChildElement("kinect");
	GetParam(setting, "enable", enable);
	if (setting && enable)
	{
		pKinect = Devices::KinectSensor::GetForCurrentView();
		if (pKinect)
		{
			XMMATRIX kinectCoord = XMMatrixRigidTransform(
				XMQuaternionRotationRollPitchYaw(-XM_PI / 12.0f, XM_PI, 0), // Orientation
				XMVectorSet(0, 0.0, 1.0f, 1.0f)); // Position
			pKinect->SetDeviceCoordinate(kinectCoord);
			//pKinect->Start();
		}
	}
#endif

}

void App::OnExit()
{
}

bool App::OnIdle()
{
	if (pLeap && !pLeap->IsAsychronize())
		pLeap->Update();

	if (pVicon && !pVicon->IsAsychronize())
		pVicon->Update();

	if (pKinect && !pKinect->IsAsychronize())
		pKinect->Update();

	for (auto& pScene : Scenes)
	{
		pScene->Update();
	}

	pDeviceResources->GetBackBufferRenderTarget().Clear(pContext.Get());
	for (auto& pScene : Scenes)
	{
		pScene->Render(pContext.Get());
	}

	pDeviceResources->Present();

	return true;
}

void App::OnDeviceLost()
{
}

void App::OnDeviceRestored()
{
}

void App::OnResize(Vector2 size)
{
	//pDeviceResources->SetLogicalSize(DeviceResources::Size(size.x,size.y));
	//auto& bb = pDeviceResources->GetBackBufferRenderTarget();
	//for (auto& scene : Scenes)
	//{
	//	scene->SetCanvas(bb);
	//}
}

path App::GetResourcesDirectory() const
{
	return m_assetsDir.wstring();
}

void App::SetResourcesDirectory(const std::wstring & dir)
{
	m_assetsDir = dir;
}
