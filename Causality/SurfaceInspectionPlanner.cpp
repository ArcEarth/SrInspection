#include "pch_bcl.h"
#include "SurfaceInspectionPlanner.h"
#include "Scene.h"
#include <Models.h>
#include <VertexTypes.h>
#include <PrimitiveVisualizer.h>
#include <Eigen\Core>
#include "Cca.h"
#include <Geometrics\csg.h>
#include <Geometrics\Primitives.h>
#include "Pointer.h"
#include "Keyboard.h"
#include "TrackerdPen.h"
#include <fstream>
#include <filesystem>

_HLSL_REGISTER_VECTOR_TYPE_(DirectX::Vector2, float, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(DirectX::Vector3, float, 1, 3);
_HLSL_REGISTER_VECTOR_TYPE_(DirectX::Vector4, float, 1, 4);

template <typename _Scalar, size_t _Dim>
inline std::ostream& operator<<(std::ostream& lhs, hlsl::xmvector<_Scalar, _Dim> v)
{
	_Scalar s[_Dim]; s << v;
	for (int i = 0; i < _Dim; i++)
		lhs << s[i] << ',';
	lhs << "\b";
	return lhs;
}


using namespace Causality;
using namespace Causality::SurfaceInspection;
using namespace DirectX::Scene;
using namespace Causality::Math;
using namespace DirectX;
using namespace DirectX::VertexTraits;
using namespace ::hlsl;
using namespace ::hlsl::alias;

REGISTER_SCENE_OBJECT_IN_PARSER(surface_inspector, SurfaceInspectionPlanner);

float g_ControlPointsRaius = 0.005f;
float g_ControlPointsConnectionRadius = 0.002;
bool  g_VisualizeNormal = false;
static constexpr size_t g_DecalResolution = 2048;

namespace Causality
{
	extern bool		g_DebugView;
}

struct TeapotPatch
{
	bool mirrorZ;
	int indices[16];
};

// Static data array defines the bezier patches that make up the teapot.
extern const TeapotPatch TeapotPatches[10];

// Static array defines the control point positions that make up the teapot.
extern const DirectX::XMVECTORF32 TeapotControlPoints[127];

template <typename _VertexType, typename _IndexType>
inline std::shared_ptr<MeshBuffer> CreateMeshBuffer(IRenderDevice* pDevice,
	const Geometrics::TriangleMesh<_VertexType, _IndexType>& mesh)
{
	auto pMesh = make_shared<MeshBuffer>();
	pMesh->CreateDeviceResources(pDevice, mesh.vertices.data(), mesh.vertices.size(), mesh.indices.data(), mesh.indices.size());
	return pMesh;
}

void SurfaceInspectionPlanner::Parse(const ParamArchive * archive)
{
	using namespace DirectX::VertexTraits;
	SceneObject::Parse(archive);

	const char* cpstr = nullptr;
	std::vector<float> cps;
	const char* mesh_name = nullptr;

	m_cursor = CoreInputs::PrimaryPointer();
	m_decalBackground = Colors::Transparent.v;
	m_decalFill = Colors::LimeGreen.v;
	m_decalStroke = Colors::Black.v;
	//m_decalStroke.A(0.8f);
	//m_decalFill.A(0.8f);

	Color color = DirectX::Colors::White;

	m_cursorMaterial = std::make_shared<PhongMaterial>();
	m_cursorMaterial->Alpha = 0.5f;

	m_isReady = false;
	m_declDirtyFalg = 0;
	int tessellation = 9;
	GetParam(archive, "tessellation", tessellation);
	GetParam(archive, "color", color);
	float patchSize = 0.1f, margin = 0.001f, z_tolerence = 0.01;

	GetParam(archive, "patch_size", patchSize);
	GetParam(archive, "margin", margin);
	GetParam(archive, "z_tolerence", z_tolerence);
	GetParam(archive, "active_fill_color", m_decalFill);
	GetParam(archive, "stroke_color", m_decalStroke);
	//GetParam(archive, "camera_hfov", );
	GetParam(archive, "projector_displacement", m_projectorDisplacement);

	//GetParamArray(archive, "control_points", cps);

	//if (cps.size() < 16 * 3)
	//	return;

	//std::copy_n(cps.data(), 3 * 16, (float*)m_patch.data());

	m_requestCancelLoading = 0;
	//m_loadingTask = concurrency::create_task([=]() {
	//	m_mesh.build();

	//	int xsize = (int)ceilf(obb.Extents.x * 2.0f / patchSize);
	//	int ysize = (int)ceilf(obb.Extents.z * 2.0f / patchSize);
	//	xsize = 0;
	//	ysize = 0;

	//	//m_mesh.transform(XMMatrixTranslationFromVector(-XMLoad(bb.Center)));

	//	using Geometrics::csg::BSPNode;

	//	std::cout << "Building mesh BSP tree..." << std::endl;

	//	uptr<BSPNode> meshNode;
	//	try
	//	{
	//		meshNode = BSPNode::create(m_mesh);
	//	}
	//	catch (const std::exception&)
	//	{
	//		return;
	//	}

	//	if (this->m_requestCancelLoading) return;

	//	Geometrics::TriangleMesh<VertexPositionNormalTexture> cubeMesh;
	//	DirectX::GeometricPrimitive::CreateCube(cubeMesh.vertices, cubeMesh.indices, 1.0f);

	//	if (this->m_requestCancelLoading) return;

	//	float zoffset = bb.Center.y;
	//	XMVECTOR sclFactor = XMVectorSet(0.9f * patchSize, 5.0 * z_tolerence, 0.9f *  patchSize, 1.0f);
	//	for (int i = 0; i < xsize; i++)
	//	{
	//		for (int j = 0; j < ysize; j++)
	//		{
	//			float xoffset = bb.Center.x - bb.Extents.x + (i + 0.5f) * patchSize;
	//			float yoffset = bb.Center.z - bb.Extents.z + (j + 0.5f) * patchSize;

	//			XMMATRIX M = XMMatrixAffineTransformation(
	//				sclFactor, XMVectorZero(), XMQuaternionIdentity(),
	//				XMVectorSet(xoffset, zoffset, yoffset, 0.0f));

	//			cubeMesh.transform(M);

	//			if (this->m_requestCancelLoading) return;

	//			// the area of interest for this index
	//			auto cubeNode = BSPNode::create(cubeMesh);

	//			if (this->m_requestCancelLoading) return;

	//			XMVECTOR det;
	//			M = XMMatrixInverse(&det, M);
	//			cubeMesh.transform(M); // transform back

	//			std::cout << "Computing mesh insection (" << i << ',' << j << ')' << std::endl;
	//			uptr<BSPNode> nodeRet;
	//			try
	//			{
	//				nodeRet.reset(Geometrics::csg::nodeSubtract(meshNode.get(), cubeNode.get()));
	//			}
	//			catch (const std::exception&)
	//			{
	//				return;
	//			}

	//			if (this->m_requestCancelLoading) return;

	//			// the segmented mesh
	//			m_fracorizedMeshes.emplace_back();
	//			try
	//			{
	//				nodeRet->convertToMesh(m_fracorizedMeshes.back());
	//			}
	//			catch (const std::exception&)
	//			{
	//				return;
	//			}

	//			if (this->m_requestCancelLoading) return;
	//		}
	//	}

	//	//m_projMesh = m_mesh; // copy the mesh
	//	//for (auto& v : m_projMesh.vertices)
	//	//{
	//	//	XMVECTOR p = get_position(v);
	//	//	p = XMVector3Rotate(p,quat);
	//	//	set_position(v, p);
	//	//}

	//	{
	//		auto pModel = new CompositionModel();
	//		this->m_factorizeModel.reset(pModel);
	//		if (this->m_requestCancelLoading) return;

	//		auto& parts = pModel->Parts;
	//		for (auto& mesh : m_fracorizedMeshes)
	//		{
	//			parts.emplace_back();
	//			auto& part = parts.back();

	//			if (this->m_requestCancelLoading) return;

	//			part.pMesh = CreateMeshBuffer(pDevice, mesh);
	//			part.Name = "factorized_patch";

	//			if (this->m_requestCancelLoading) return;

	//			CreateBoundingBoxesFromPoints(part.BoundBox, part.BoundOrientedBox,
	//				mesh.vertices.size(), &mesh.vertices[0].position, sizeof(TVertex));

	//			if (this->m_requestCancelLoading) return;
	//		}
	//		pModel->CreateBoundingGeometry();

	//		if (this->m_requestCancelLoading) return;

	//		auto pVisual = new VisualObject();
	//		pVisual->Scene = this->Scene;
	//		pVisual->SetRenderModel(pModel);

	//		if (this->m_requestCancelLoading) return;
	//		{
	//			this->AddChild(pVisual);
	//			std::lock_guard<std::mutex>(this->Scene->ContentMutex());
	//			this->Scene->SignalCameraCache();
	//		}
	//		//VisualObject::m_pRenderModel = pModel;
	//	}

	//	m_isReady = true;
	//});
}

// Gernerate a roll-patch-roll rotation that sort ext into 
inline XMVECTOR XM_CALLCONV GetSortRotation(XMFLOAT3& extent)
{
	XMVECTOR q = XMQuaternionIdentity();
	Vector3 ext = extent;

	if (ext.x < ext.y)
	{
		std::swap(ext.x, ext.y);
		q = XMQuaternionMultiply(XMQuaternionRotationRoll(XM_PIDIV2), q);
	}
	if (ext.y < ext.z)
	{
		std::swap(ext.y, ext.z);
		q = XMQuaternionMultiply(XMQuaternionRotationPatch(XM_PIDIV2), q);
	}
	if (ext.x < ext.y)
	{
		std::swap(ext.x, ext.y);
		q = XMQuaternionMultiply(XMQuaternionRotationRoll(XM_PIDIV2), q);
	}

	XMVECTOR rotated = XMVector3Rotate(XMLoad(ext), q);
	rotated = XMVectorAbs(rotated);
	extent = ext;
	return q;
}

void SortOrientedBoundingBox(BoundingOrientedBox& obb)
{
	auto q = GetSortRotation(obb.Extents);
	auto oq = XMLoad(obb.Orientation);
	q = XMQuaternionMultiply(q, oq);
	XMStore(obb.Orientation, q);
}

void SurfaceInspectionPlanner::ExtractWorkloadMesh(const IModelNode* pModel)
{
	if (pModel)
		CopyMesh(m_mesh, pModel);
	else
		TesselateBezeirPatch(10, Color(Colors::AliceBlue.f));

	BoundingBox bb, uvbb;
	BoundingOrientedBox obb;

	using TVertex = decltype(m_mesh.vertices[0]);
	using TIndex = decltype(m_mesh.indices[0]);

	XMVECTOR uvmin = XMVectorReplicate(100.0f);
	XMVECTOR uvmax = XMVectorReplicate(-100.0f);
	for (auto& v : m_mesh.vertices)
	{
		uvmin = XMVectorMin(get_uv(v), uvmin);
		uvmax = XMVectorMax(get_uv(v), uvmax);
	}
	uvmax = XMVectorReciprocal(uvmax - uvmin);
	uvmin = -uvmin;
	for (auto& v : m_mesh.vertices)
		set_uv(v, XMVectorMultiplyAdd(get_uv(v), uvmax, uvmin));

	// the BoundingOrientedBox is computed from an 3x3 svd, (pca of the vertices)
	CreateBoundingBoxesFromPoints(bb, obb,
		m_mesh.vertices.size(), &m_mesh.vertices[0].position, sizeof(TVertex));
	SortOrientedBoundingBox(obb);

	XMVECTOR t = -XMLoad(obb.Center);
	XMVECTOR q = XMQuaternionConjugate(XMLoad(obb.Orientation));
	uvmin = XMVectorReplicate(0.5f);
	uvmax = uvmin * XMVectorReciprocal(XMLoad(obb.Extents));
	for (auto& v : m_mesh.vertices)
	{
		auto p = get_position(v) + t;
		p = XMVector3Rotate(p, q);
		p = XMVectorAndInt(p, XMVECTORU32{ 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000 }.v);
		p = XMVectorMultiplyAdd(p, uvmax, uvmin);
		set_uv(v, p);
	}

	m_mesh.build();
	m_previwPatch.m_opticity = 0.5f;
	m_previwPatch.setSurface(&m_mesh);
	CreateDecalDeviceResources(bb, obb);
}

void SurfaceInspectionPlanner::CreateDecalDeviceResources(DirectX::BoundingBox &bb, DirectX::BoundingOrientedBox &obb)
{
	auto pDevice = this->Scene->GetRenderDevice();
	auto pD2dContext = this->Scene->Get2DContext();

	{
		auto pModel = new MonolithModel();
		pModel->pMesh = CreateMeshBuffer(pDevice, m_mesh);
		pModel->SetName("whole_patch");
		pModel->BoundBox = bb;
		pModel->BoundOrientedBox = obb;
		//VisualObject::m_pRenderModel = pModel;

		m_decal = RenderableTexture2D(pDevice, g_DecalResolution, g_DecalResolution, DXGI_FORMAT_B8G8R8A8_UNORM, 1, 0, true);
		m_decal.CreateD2DBitmapView(pD2dContext);

		auto declMat = make_shared<PhongMaterial>();
		declMat->DiffuseMap = m_decal;
		declMat->UseAlphaDiscard = true;

		pModel->pMaterial = std::move(declMat);
		UpdateDecalGeometry(Scene->Get2DFactory());

		m_decalModel.reset(pModel);
	}
}

void SurfaceInspectionPlanner::CopyMesh(TriangleMeshType& mesh, const IModelNode* pNode)
{
	using DirectX::Scene::DefaultStaticModel;
	using DirectX::Scene::DefaultSkinningModel;
	using namespace DirectX::VertexTraits;

	auto pModel = dynamic_cast<const DefaultStaticModel*>(pNode);
	if (pModel)
	{
		auto& vertics = mesh.vertices;
		auto& indices = mesh.indices;

		TriangleMeshType::VertexType vt;
		for (auto& v : pModel->Vertices)
		{
			convert_vertex(v, vt);
			vertics.push_back(vt);
		}
		for (auto& f : pModel->Facets)
		{
			for (int i = 0; i < 3; ++i)
				indices.push_back(f[i]);
		}
	}
}

void SurfaceInspectionPlanner::TesselateBezeirPatch(int tessellation, DirectX::SimpleMath::Color &color)
{
	//auto& patch = TeapotPatches[9];
	//for (int i = 0; i < std::size(patch.indices); i++)
	//{
	//	m_patch[i] = TeapotControlPoints[patch.indices[i]].v;
	//}

	bool succ = Geometrics::triangluate(m_bezeirPatch, m_mesh, tessellation);

	for (auto& cp : m_bezeirPatch) { cp.x = -cp.x; cp.z = -cp.z; }
	succ = Geometrics::triangluate(m_bezeirPatch, m_mesh, tessellation);

	for (auto& cp : m_bezeirPatch) cp.x = -cp.x;
	succ = Geometrics::triangluate(m_bezeirPatch, m_mesh, tessellation, true);

	for (auto& cp : m_bezeirPatch) { cp.x = -cp.x; cp.z = -cp.z; }
	succ = Geometrics::triangluate(m_bezeirPatch, m_mesh, tessellation, true);

	for (auto& v : m_mesh.vertices)
	{
		DirectX::VertexTraits::set_color(v, color);
	}

	m_mesh.flip();

}

InspectionPatch * SurfaceInspectionPlanner::AddInspectionPatch(FXMVECTOR uv, int fid)
{
	m_isPatches.push_back(m_previwPatch);
	auto& patch = m_isPatches.back();
	CaculateCameraPath();
	patch.m_opticity = 0.8f;
	//patch.setSurface(&m_mesh);
	//patch.CurvtureDistribution = m_previwPatch.CurvtureDistribution;
	//patch.SetSurfaceCurveToUVRect(uv, m_previwPatch.m_uvExtent);
	//auto pFactory = this->Scene->Get2DFactory();
	//patch.UpdateGeometry(pFactory);
	//patch.CaculateCameraFrustum();
	//CaculateCameraPath();
	return &patch;
}

void SurfaceInspectionPlanner::CaculateCameraPath()
{
	m_isCameraPath.push_back(m_isCameraPath.size());
	auto dis = [this](int i, int j) {return Vector3::Distance(m_isPatches[i].m_cameraFrustum.Origin, m_isPatches[j].m_cameraFrustum.Origin); };

	vector<int> path(m_isCameraPath);
	auto& spath = m_isCameraPath;
	int n = m_isCameraPath.size();
	float minDis = std::numeric_limits<float>::max();
	vector<BOOL> visited(n, false);
	visited[m_isCameraPath[0]] = true;

	function<void(int, int)> search_recur = [&](int k, float sum) -> void
	{
		if (k == n && sum < minDis)
		{
			spath = path; minDis = sum;
			return;
		}
		auto citr = path.begin() + k;
		for (int i = 0; i < n; i++)
		{
			if (!visited[i])
			{
				path[k] = i;
				visited[i] = true;
				float newS = sum + dis(path[k - 1], path[k]);
				if (newS < minDis)
					search_recur(k + 1, newS);
				visited[i] = false;
			}
		}
	};

	search_recur(1, .0f);
}

void PrintVector3PBRT(std::ostream& os, DirectX::FXMVECTOR q)
{
	Vector3 saxis = q;
	os << saxis.x << ' ' << saxis.y << ' ' << saxis.z;
}

void PrintRotationPBRT(std::ostream& os, DirectX::FXMVECTOR q)
{
	float angle; XMVECTOR axis;
	XMQuaternionToAxisAngle(&axis, &angle, q);
	os << XMConvertToDegrees(angle) << ' ';
	PrintVector3PBRT(os, axis);
}

namespace pbrt
{
	std::ostream& operator <<(std::ostream& os, DirectX::Vector3 v)
	{
		PrintVector3PBRT(os, v); return os;
	}
	std::ostream& operator <<(std::ostream& os, DirectX::Quaternion q)
	{
		PrintRotationPBRT(os, q); return os;
	}
}

void SurfaceInspectionPlanner::PrintPlan(std::ostream & os) const
{
	using namespace std;
	os << "#Inspection Plan:" << m_workloadObj->Name << endl;
	os << endl;
	os << "#Model world transform:" << endl;
	os << "Translate ";
	PrintVector3PBRT(os, m_workloadObj->GetPosition());
	os << endl << "Rotate ";
	PrintRotationPBRT(os, m_workloadObj->GetOrientation());
	os << endl << "Scale ";
	PrintVector3PBRT(os, m_workloadObj->GetScale());
	os << endl;
	os << "#Camera Focus: " << 8.0f << endl;
	int n = m_isPatches.size();
	for (int i = 0; i < n; i++)
	{
		auto& patch = m_isPatches[m_isCameraPath[i]];
		auto& frustum = patch.m_cameraFrustum;
		os << "#Camera " << i << endl;
		os << "Translate ";
		PrintVector3PBRT(os, Vector3(frustum.Origin));
		os << endl << "Rotate ";
		PrintRotationPBRT(os, Quaternion(frustum.Orientation));
		os << endl;
	}
}

int str_replace_first(string& source, const string& partten, const string& rep)
{
	size_t index = 0;
	/* Locate the substring to replace. */
	index = source.find(partten, index);
	if (index == std::string::npos) return index;

	/* Make the replacement. */
	source.replace(index, partten.size(), rep);

	return index;
}

void SurfaceInspectionPlanner::GenerateScript() const
{
	namespace sys = std::experimental::filesystem;
	auto sctipts_root = sys::current_path().parent_path() / "Assets" / "Scripts";
	std::ifstream fctemp(sctipts_root / "shot.pbrt.template");
	std::string shoot_template((std::istreambuf_iterator<char>(fctemp)), std::istreambuf_iterator<char>());
	std::ifstream fdtemp(sctipts_root / "dart.pbrt.template");
	std::string dart_template((std::istreambuf_iterator<char>(fdtemp)), std::istreambuf_iterator<char>());
	fctemp.close(); fdtemp.close();

	string wkName = m_workloadObj->RenderModel()->Name();
	auto workload_root = sctipts_root / ("plan_" + wkName);
	auto pbrtbin = sys::current_path().parent_path() / "pbrt";
	auto meshobj = sys::current_path().parent_path() / "Assets" / "Meshes" / (wkName + ".obj");

	if (sys::exists(wkName))
		sys::remove_all(wkName);
	sys::create_directory(workload_root);

	std::ofstream dark_specification_file(workload_root / "dart.pbrt");
	// To-do add darts
	dark_specification_file.close();

	str_replace_first(shoot_template, "{WorkloadMeshPbrtFileName}", wkName + ".pbrt");
	str_replace_first(shoot_template, "{DartSpecificationPbrtFileName}", "dart.pbrt");

	std::ofstream excute_bat(workload_root / "excute.bat");
	excute_bat << "@set PATH=%PATH%;" << pbrtbin << std::endl;
	excute_bat << "@obj2pbrt.exe " << meshobj << ' ' << (workload_root/(wkName + ".pbrt")) << std::endl;
	int n = m_isPatches.size();
	char buffer[256];
	for (int i = 0; i < n; i++)
	{
		string ctemp = shoot_template;
		auto& patch = m_isPatches[m_isCameraPath[i]];
		auto& frustum = patch.m_cameraFrustum;
		int printed = sprintf_s(buffer, "%f %f %f", frustum.Origin.x, frustum.Origin.y, frustum.Origin.z);
		buffer[printed] = 0;
		str_replace_first(ctemp, "{Eye}", buffer);

		Vector3 focus = { 0,0,m_cameraFrustum.Far };
		focus = XMVector3Rotate(focus, XMLoad(frustum.Orientation));
		focus += frustum.Origin;
		printed = sprintf_s(buffer, "%f %f %f", focus.x, focus.y, focus.z);
		buffer[printed] = 0;
		str_replace_first(ctemp, "{Focus}", buffer);

		Vector3 up = { 0,1,0 };
		up = XMVector3Rotate(up, XMLoad(frustum.Orientation));
		printed = sprintf_s(buffer, "%f %f %f", up.x, up.y, up.z);
		buffer[printed] = 0;
		str_replace_first(ctemp, "{Up}", buffer);

		for (int lp = 0; lp < 4; lp++)
		{
			string cstemp = ctemp;
			printed = sprintf_s(buffer, "%d", lp);
			buffer[printed] = 0;
			str_replace_first(cstemp, "{ProjectPattern}", buffer);

			printed = sprintf_s(buffer, "camera_%d_pattern_%d.pbrt", i, lp);
			buffer[printed] = 0;

			std::ofstream shoot_file(workload_root / buffer);
			shoot_file << cstemp;
			shoot_file.close();

			excute_bat << "@echo \">>>>>>>>>>>>> Rendering Image " << buffer << " >>>>>>>>>>>>>\"" << std::endl;
			excute_bat << "@call pbrt_tiff.bat " << buffer << std::endl;
		}
	}
	excute_bat << "@mkdir image" << std::endl;
	excute_bat << "@move " << workload_root <<"\\*.tiff "<< workload_root <<"\\image" << std::endl;
	excute_bat.close();

	system(("explorer " + workload_root.string()).c_str());
	//system((workload_root / "excute.bat").string().c_str());
}

SurfaceInspectionPlanner::SurfaceInspectionPlanner()
{
	static const size_t g_MaxPatchCount = 64;
	m_state = State_Initializing;
	m_pen = nullptr;
	m_workloadObj = nullptr;
	m_isPatches.reserve(g_MaxPatchCount);
	this->OnParentChanged += [this](SceneObject* _this, SceneObject* old_parent)
	{
		auto vo = Parent()->As<VisualObject>();
		if (vo)
		{
			this->SetWorkload(vo);
		}
	};

	CoreInputs::PrimaryKeyboard()->KeyDown += [this](const KeyboardEventArgs& arg)
	{
		if (arg.Key == VK_RETURN && this->m_state == State_Idle && this->m_isHit)
		{
			auto its = this->m_isInfo;
			auto v = this->m_mesh.persudo_vertex(its.facet, its.barycentric);
			this->AddInspectionPatch(get_uv(v), its.facet);
		}
		if (arg.Key == 'P')
		{
			namespace sys = std::tr2::sys;
			std::string fileName = this->m_workloadObj->Name + ".txt";
			auto path = sys::current_path() / fileName;
			std::cout << "Printing Inspection Plan >> " << path.string() << std::endl;
			this->PrintPlan(std::cout);
			std::ofstream planfile(path);
			this->PrintPlan(planfile);
			planfile.close();
			GenerateScript();
			std::cout << "Printing finished" << std::endl;
		}
	};
}

SurfaceInspectionPlanner::~SurfaceInspectionPlanner()
{
	m_requestCancelLoading = 1;

	//if (!m_loadingTask.is_done())
	//	m_loadingTask.wait();

	//m_decal.BitmapView()->Release();
	m_decal.Reset();
	//std::cout << "Decal destroyed" << std::endl;
	//auto pModel = static_cast<MonolithModel*>(m_pModel.release());
	//if (pModel) delete pModel;
}

bool SurfaceInspectionPlanner::IsVisible(const BoundingGeometry & viewFrustum) const
{
	return true;
}

void SurfaceInspectionPlanner::AddChild(SceneObject * child)
{
	SceneObject::AddChild(child);
	if (!m_pen)
		m_pen = child->As<TrackedPen>();
	if (child->Name == "workload")
	{
		auto pVO = child->As<VisualObject>();
		SetWorkload(pVO);
	}
}

void SurfaceInspectionPlanner::SetWorkload(VisualObject * pVO)
{
	std::cout << "[SIP] Workload set to object [" << pVO->Name << ']' << std::endl;
	m_state = State_Initializing;

	auto pModel = pVO ? pVO->RenderModel() : nullptr;
	if (!pModel)
	{
		MessageBoxA(nullptr, "Workload file dose not exist", "Failed to load file", 0);
		std::terminate();
	}

	SetWorkloadFillView(pModel, pVO);

	m_workloadObj = pVO;
	if (pModel)
	{
		m_loadingTask = concurrency::create_task([=]() {
			this->ExtractWorkloadMesh(pModel);
			m_state = State_Idle;
			m_isReady = true;
		});
	}
}

void SurfaceInspectionPlanner::SetWorkloadFillView(DirectX::Scene::IModelNode * pModel, Causality::VisualObject * pVO)
{
	auto bb = pModel->GetOrientedBoundingBox();
	SortOrientedBoundingBox(bb);
	float scl = 1.0f / bb.Extents.x;

	(Vector3&)(bb.Extents) *= scl;

	XMVECTOR tsl = XMLoad(bb.Center) * scl;
	XMVECTOR oq = XMLoad(bb.Orientation);

	XMMATRIX rotm = XMMatrixIdentity();
	XMVECTOR quat = XMQuaternionRotationPatch(-XM_PIDIV2);
	quat = XMQuaternionMultiply(XMQuaternionConjugate(XMLoad(bb.Orientation)), quat);
	tsl = XMVector3Rotate(-tsl, quat) + XMVectorSet(0, bb.Extents.z, 0, 0);

	pVO->SetPosition(tsl);
	pVO->SetOrientation(quat);
	pVO->SetScale(Vector3(scl));

	DirectX::BoundingBox bbx;
	pVO->GetBoundingBox(bbx);
}

void SurfaceInspectionPlanner::Update(time_seconds const & time_delta)
{
	using namespace DirectX::VertexTraits;
	m_isHit = false;
	if (m_isReady && m_pen)
	{
		// These position / directions are in local coordinate
		XMVECTOR pos = m_pen->GetTipPosition();
		XMVECTOR dir = m_pen->GetTipDirection();

		bool hit = HitTest(pos, dir);
		if (hit)
		{
			if (m_state == State_Idle) // Pressing Button 
			{
				auto pv = m_mesh.persudo_vertex(m_isInfo.facet, m_isInfo.barycentric);
				if (m_pen->IsInking())
				{
					auto patch = TrySelectInspectionPatch(get_uv(pv), m_isInfo.facet);
					if (patch)
						SurfacePatchDragBegin(patch);
					else
						SurfaceSketchBegin();
				}
				else
				{
					m_previwPatch.m_isInfo = m_isInfo; // copy the intersection info
					m_previwPatch.SetSurfaceCurveToUVRect(get_uv(pv), m_previwPatch.m_uvExtent);
					m_previwPatch.CaculateCameraFrustum();
					m_declDirtyFalg = 1;
				}
			}

			switch (m_state)
			{
			case State_DrawingPatch:
				SrufaceSketchUpdate(pos, dir);
				break;
			case State_DragingPatch:
				SurfacePatchDragUpdate(pos);
				break;
			case State_Idle:
			default:
				break;
			}
		}

		if (m_state == State_DrawingPatch && (!m_pen->IsInking() || !hit))
		{
			SurfaceSketchEnd();
		}
		else if (m_state == State_DragingPatch && (!m_pen->IsInking() || !hit))
		{
			SurfacePatchDragEnd();
		}

	}

}

bool XM_CALLCONV SurfaceInspectionPlanner::HitTest(FXMVECTOR pos_world, FXMVECTOR dir_world)
{
	using namespace DirectX::VertexTraits;
	if (!m_workloadObj || !m_isReady) return false;
	XMMATRIX invworld = m_workloadObj->GetGlobalTransform().Inversed().TransformMatrix();

	m_isHit = false;
	XMVECTOR pos = XMVector3Transform(pos_world, invworld);
	XMVECTOR dir = XMVector3TransformNormal(dir_world, invworld);
	dir = XMVector3Normalize(dir);
	m_castRay = DirectX::Ray(pos, dir);

	//std::vector<Geometrics::MeshRayIntersectionInfo> intersecs;
	float dis;

	// fast rejection path
	//if (!m_decalModel->GetOrientedBoundingBox().Intersects(pos, dir, dis))
	//	return false;
	//if (!m_decalModel->GetBoundingBox().Intersects(pos, dir, dis))
	//	return false;

	m_isInfo = m_mesh.first_intersect(pos, dir);
	m_isHit = m_isInfo.facet >= 0;
	//if (!intersecs.empty())
	//{
	//	m_isHit = true;
	//	m_isInfo = intersecs[0];
	//}
	return m_isHit;
}

int DrawBVH(const TriangleMeshType::triangle_bvh_t& tree, int node, int maxDepth)
{
	if (maxDepth <= 0) return 0;
	auto&& aabb = tree.get_volumn(node);
	XMVECTOR color = (aabb.intersected ? Colors::Green : Colors::Orange);
	auto children = tree.get_children(node);

	if (aabb.intersected)
		color = DirectX::XMVectorSetW(color, 0.2f);
	else
		color = DirectX::XMVectorSetW(color, 0.4f);

	if (children.empty())
		color = (aabb.intersected ? Colors::Lime : Colors::Orange);

	DirectX::BoundingGeometry geo(aabb.get_dxbox());
	DrawGeometryOutline(geo, color);

	int level = 100;
	if (aabb.intersected)
	{
		level = maxDepth;
		for (auto& c : children)
			level = std::min(level, DrawBVH(tree, c, maxDepth - 1));
	}
	return level;
}

int DrawBVH2(const TriangleMeshType::triangle_bvh_t& tree, int node, DirectX::Ray ray)
{
	std::vector<int> stck; stck.reserve(20);
	int nid = node;
	while (nid >= 0)
	{
		stck.push_back(nid);
		nid = tree.get_parent(nid);
	}
	int errornode = -1;
	for (int i = stck.size() - 1; i >= 0; --i)
	{
		if (errornode < 0 && !tree.get_volumn(stck[i]).intersected)
			errornode = i;
		if (errornode >= 0)
		{
			int node = stck[i];
			DirectX::BoundingGeometry geo(tree.get_volumn(node).get_dxbox());
			DrawGeometryOutline(geo, Colors::Red);
		}
	}
	return errornode;
}

void SurfaceInspectionPlanner::Render(IRenderContext * pContext, IEffect * pEffect)
{
	if (m_declDirtyFalg)
		DrawDecal(Scene->Get2DContext());

	if (m_decalModel)
	{
		auto world = m_workloadObj->GlobalTransformMatrix();
		m_decalModel->Render(pContext, world, pEffect);

		bool shoulddraw = g_DebugView || m_isHit || m_isPatches.size() > 0;

		if (shoulddraw)
		{
			auto& darwer = Visualizers::g_PrimitiveDrawer;
			darwer.SetWorld(world);
			darwer.Begin();

			const int g_debug_bvh = false;
			if (g_DebugView && g_debug_bvh)
			{
				int level = DrawBVH(m_mesh.triangle_bvh, m_mesh.triangle_bvh.root(), 20);

				int err = DrawBVH2(m_mesh.triangle_bvh, m_isInfo.facet, m_castRay);

				if (err != -1)
				{
					auto tri = m_mesh.facet(m_isInfo.facet);

					XMVECTOR v0 = get_position(m_mesh.vertices[tri[0]]);
					XMVECTOR v1 = get_position(m_mesh.vertices[tri[1]]);
					XMVECTOR v2 = get_position(m_mesh.vertices[tri[2]]);
					darwer.DrawLine(v0, v1, Colors::Red);
					darwer.DrawLine(v1, v2, Colors::Red);
					darwer.DrawLine(v2, v0, Colors::Red);
				}
			}

			if (m_isHit)
				DrawPatchCamera(m_previwPatch);

			for (auto& patch : m_isPatches)
			{
				DrawPatchCamera(patch);
			}

			for (int i = 1; i < m_isCameraPath.size(); i++)
			{
				int pcpid = m_isCameraPath[i - 1];
				int cpid = m_isCameraPath[i];
				auto prev = XMLoad(m_isPatches[pcpid].m_cameraFrustum.Origin);
				auto curr = XMLoad(m_isPatches[cpid].m_cameraFrustum.Origin);
				darwer.DrawLine(prev, curr, DirectX::Colors::Blue);
			}
			darwer.End();
		}
	}

	if (m_pen)
	{
		//RenderCursor(pContext, pEffect);
	}

	if (g_DebugView && pEffect)
	{
		DrawBezeirPatchControlPoints();
	}
}

void SurfaceInspectionPlanner::DrawPatchCamera(InspectionPatch& patch)
{
	BoundingGeometry geo(patch.BoundingBox);
	XMVECTOR color = patch.Valiad ? Colors::Lime.v : Colors::Red.v;
	//DrawGeometryOutline(geo, color);
	geo = patch.m_cameraFrustum;
	DrawGeometryOutline(geo, color);
	geo = patch.BoundingBox;
	if (g_DebugView)
	{
		DrawGeometryOutline(geo, color);

		auto& surface = patch.surface();
		for (auto& f : patch.m_areaFacets)
		{
			int fid = f.first;
			auto cont = f.second.containment;
			color = cont == 2 ? Colors::Lime : cont == 1 ? Colors::Purple : Colors::Red;
			for (int i = 0; i < 3; i++)
			{
				auto v0 = get_position(surface.vertex(fid, i));
				auto v1 = get_position(surface.vertex(fid, (i + 1) % 3));
				Visualizers::g_PrimitiveDrawer.DrawLine(v0, v1, color);
			}
		}
	}
}

RenderFlags SurfaceInspectionPlanner::GetRenderFlags() const { return RenderFlags::SpecialEffects; }

void XM_CALLCONV SurfaceInspectionPlanner::UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection) {}

void SurfaceInspectionPlanner::DrawBezeirPatchControlPoints()
{
	auto& drawer = DirectX::Visualizers::g_PrimitiveDrawer;
	auto world = this->GlobalTransformMatrix();
	//geo.Transform(geo, GlobalTransformMatrix());
	Color color = DirectX::Colors::LimeGreen.v;

	drawer.SetWorld(world);

	if (m_decalModel && g_VisualizeNormal)
	{
		drawer.Begin();
		using namespace DirectX::VertexTraits;
		for (auto& v : m_mesh.vertices)
		{
			XMVECTOR pv = get_position(v);
			XMVECTOR pnv = get_normal(v);
			pnv = pv + 0.01 * pnv;
			drawer.DrawLine(pv, pnv, DirectX::Colors::Red.v);

			pnv = get_tangent(v);
			pnv = pv + 0.01 * pnv;
			drawer.DrawLine(pv, pnv, color);
		}

		drawer.End();
	}

	for (int i = 0; i < 4; i++)
	{
		drawer.DrawSphere(m_bezeirPatch.control_point(i, 0), g_ControlPointsRaius, color);
		drawer.DrawSphere(m_bezeirPatch.control_point(i, 3), g_ControlPointsRaius, color);
	}

	for (int i = 1; i < 3; i++)
	{
		drawer.DrawSphere(m_bezeirPatch.control_point(0, i), g_ControlPointsRaius, color);
		drawer.DrawSphere(m_bezeirPatch.control_point(3, i), g_ControlPointsRaius, color);
	}

	for (int i = 0; i < 3; i++)
	{
		drawer.DrawCylinder(m_bezeirPatch.control_point(i, 0), m_bezeirPatch.control_point((i + 1) % 4, 0), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(i, 3), m_bezeirPatch.control_point((i + 1) % 4, 3), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(0, i), m_bezeirPatch.control_point(0, (i + 1) % 4), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(3, i), m_bezeirPatch.control_point(3, (i + 1) % 4), g_ControlPointsConnectionRadius, color);
	}

	color *= 1.2;
	color.A(1.0f);
	for (int i = 1; i < 3; i++)
	{
		drawer.DrawCylinder(m_bezeirPatch.control_point(i, 0), m_bezeirPatch.control_point(i, 1), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(i, 1), m_bezeirPatch.control_point(i, 2), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(i, 2), m_bezeirPatch.control_point(i, 3), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(0, i), m_bezeirPatch.control_point(1, i), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(1, i), m_bezeirPatch.control_point(2, i), g_ControlPointsConnectionRadius, color);
		drawer.DrawCylinder(m_bezeirPatch.control_point(2, i), m_bezeirPatch.control_point(3, i), g_ControlPointsConnectionRadius, color);
	}
}

void SurfaceInspectionPlanner::RenderCursor(IRenderContext * pContext, IEffect * pEffect)
{
	auto& drawer = DirectX::Visualizers::g_PrimitiveDrawer;

	XMVECTOR pos = m_pen->GetTipPosition();
	XMVECTOR dir = m_pen->GetTipDirection();

	//std::cout << "dir = " << Vector3(dir) << std::endl;
	//std::cout << "orientation = " << m_pen->GetOrientation() << std::endl;

	XMVECTOR color = Colors::Yellow.v;

	if (m_state == State_DrawingPatch)
		color = Colors::LimeGreen.v;
	else if (m_state == State_DragingPatch)
		color = Colors::Red.v;
	else if (m_state == State_Initializing)
		color = Colors::Purple.v;

	float alpha = 0.5f, scl = 1.0f;
	XMMATRIX world = XMMatrixIdentity();

	if (m_isHit)
	{
		world = m_workloadObj->GlobalTransformMatrix();
		alpha = 1.0f;
		//color = Colors::Red;
		scl /= 20.0f;
		pos = m_isInfo.position;
	}

	float length = 0.01f / scl;
	float radius = 0.0035f / scl;

	m_cursorMaterial->DiffuseColor = color;
	m_cursorMaterial->AmbientColor = color;
	m_cursorMaterial->Alpha = alpha;
	//m_cursorMaterial->SetupEffect(pEffect);

	color = XMVectorSetW(color, alpha);

	drawer.SetWorld(world);
	drawer.DrawSphere(pos, radius, color);
	//drawer.DrawCylinder(pos, pos + dir * length, radius * 0.5, color);
}

inline D2D1_COLOR_F XM_CALLCONV GetD2DColor(const Color& color)
{
	D2D1_COLOR_F cf = reinterpret_cast<const D2D1_COLOR_F&>(color);
	return cf;
}

inline D2D1_POINT_2F XM_CALLCONV GetD2DPoint(const Vector2& v)
{
	D2D1_POINT_2F dv = reinterpret_cast<const D2D1_POINT_2F&>(v);
	return dv;
}



void SurfaceInspectionPlanner::UpdateDecalGeometry(I2DFactory* pFactory)
{
	m_previwPatch.SetSurfaceCurveToUVRect(m_previwPatch.m_uvCenter, m_previwPatch.m_uvExtent);
	m_previwPatch.UpdateGeometry(pFactory);
	m_previwPatch.UpdateDecalTransformMatrix();
	m_declDirtyFalg = 1;
}

InspectionPatch * SurfaceInspectionPlanner::TrySelectInspectionPatch(FXMVECTOR uv, int fid)
{
	for (auto& patch : m_isPatches)
	{
		if (patch.uvBoundry().contains2D(uv))
			return &patch;
	}
	return nullptr;
}

void SurfaceInspectionPlanner::DrawDecal(I2DContext *pContext)
{
	D2D1_COLOR_F color;
	color = { .2f,0.7f,.2f,1.0f };

	if (!m_brush)
		ThrowIfFailed(pContext->CreateSolidColorBrush(color, &m_brush));

	//color = { .0f,.0f,.0f,.0f };
	pContext->SetTarget(m_decal);
	pContext->BeginDraw();
	pContext->Clear(GetD2DColor(m_decalBackground));

	if (m_isHit)
	{
		auto pFactory = this->Scene->Get2DFactory();
		m_previwPatch.UpdateGeometry(pFactory); // reset the histogram
		m_previwPatch.DrawDecal(pContext, m_brush.Get());
	}
	for (auto& patch : m_isPatches)
		patch.DrawDecal(pContext, m_brush.Get());

	ThrowIfFailed(pContext->EndDraw());
	pContext->SetTarget(nullptr);

	m_declDirtyFalg = 0;
}

void SurfaceInspectionPlanner::SurfaceSketchBegin()
{
	if (!m_mesh.empty())
	{
		m_isPatches.emplace_back();
		m_isPatches.back().setSurface(&m_mesh);
		m_state = State_DrawingPatch;
	}
}

void SurfaceInspectionPlanner::SrufaceSketchUpdate(XMVECTOR pos, XMVECTOR dir)
{
	static float g_contactThred = 0.05f;
	if (m_mesh.empty()) return;

	bool touching = false;
	// Find closest point on mesh using pen direction
	vector<Geometrics::MeshRayIntersectionInfo> interInfos;
	pos -= dir * TrackedPen::TipLength * 0.5f / Parent()->GetScale().x;
	m_mesh.intersect(pos, dir, &interInfos);

	if (interInfos.size() == 0) {
		std::cout << "Pen not touching; no intersections" << std::endl;
		return;
	}

	auto& info = interInfos[0];

	float shortestDist = info.distance;

	// this is cm
	if (shortestDist < g_contactThred) {
		// touching
		auto & patch = m_isPatches.back();
		//auto& curve = patch.boundry();
		patch.append(info.position, info.facet);

		auto& uvCurve = patch.uvBoundry();
		if (uvCurve.size() > 2)
		{
			auto uvs = patch.uvBoundry().sample(std::min((int)uvCurve.size(), 100));
			//UpdateRenderGeometry(uvs, Vector2(g_DecalResolution));
		}
	}
}

void SurfaceInspectionPlanner::SurfaceSketchEnd()
{
	m_state = State_Idle;
	auto& patch = m_isPatches.back();
	auto& curve = patch.boundry();
	if (curve.empty())
	{
		m_isPatches.pop_back();
	}

	patch.closeLoop();

	//curve.append(curve[0]);
	//auto curveMap = Eigen::Matrix<float, -1, 4, Eigen::RowMajor>::MapAligned((float*)curve.data(), curve.size(), 4);
	//Eigen::laplacianSmooth(curveMap, 0.8);
	//curve.updateLength();

	//if (curve.size() > 100)
	//{
	//	curve.resample(100);
	//}
	//curve.smooth(0.8f, 4);
}

void SurfaceInspectionPlanner::SurfacePatchDragBegin(InspectionPatch *patch)
{
	if (m_isPatches.empty())
		return;
	m_state = State_DragingPatch;
	m_activePatch = patch;
}

void SurfaceInspectionPlanner::SurfacePatchDragUpdate(FXMVECTOR pos)
{
	m_activePatch;
}

void SurfaceInspectionPlanner::SurfacePatchDragEnd()
{
	m_state = State_Idle;
	m_activePatch = nullptr;
}

InspectionPatch::InspectionPatch()
{
	Valiad = false;
	m_uvRotation = .0f;
	m_opticity = 1.0f;

	m_defaultCameraFrustum.Far = 8.0f;
	m_defaultCameraFrustum.Near = 1.0f;
	// it is actually half fov
	float hfov = tanf(XMConvertToRadians(16)); //10
	float vfov = tanf(XMConvertToRadians(12)); //7.5
	m_defaultCameraFrustum.RightSlope = hfov;
	m_defaultCameraFrustum.LeftSlope = -hfov;
	m_defaultCameraFrustum.TopSlope = vfov;
	m_defaultCameraFrustum.BottomSlope = -vfov;

	m_decalTransform = D2D1::Matrix3x2F::Identity();

	DecalSize = Vector2(g_DecalResolution);
	// 1/(physical dimension) * sqrt(2), so that the uv rect scales up and misalign with 45 degree  will still contains in
	float phydim = 50.0f;
	float minfov = std::fminf(hfov, vfov);
	ZTolerance = 0.16f; //0.14f
	m_uvExtent = 1 / (phydim * sqrtf(2)) * m_defaultCameraFrustum.Far * Vector2(minfov, minfov);
}

//InspectionPatch::InspectionPatch(const InspectionPatch & rhs)
//{
//	//m_surface = rhs.m_surface;
//	//m_uvCurve = rhs.m_uvCurve;
//	//m_fids = rhs.m_fids;
//	//CurvtureDistribution = rhs.CurvtureDistribution;
//	//m_areaFacets = rhs.m_areaFacets;
//	//m_areaVertices = rhs.m_areaVertices;
//	//m_averageNormal = rhs.m_averageNormal;
//	//BoundingBox = rhs.BoundingBox;
//	//m_cameraFrustum = rhs.m_cameraFrustum;
//	//cptr<ID2D1Factory> pFactory;
//	//rhs.m_deaclGeometry->GetFactory(&pFactory);
//	//UpdateGeometry(pFactory.Get());
//	//CaculateCameraFrustum();
//}

bool InspectionPatch::CaculateCameraFrustum()
{
	using namespace DirectX::VertexTraits;
	std::vector<XMVECTOR, XMAllocator> positions;
	std::vector<XMVECTOR, XMAllocator> normals;

	// Positions contains vertices & persudo-vertex added in the boundry
	CaculateVerticsInPatch(m_isInfo.facet, BoundingBox, positions);

	if (positions.size() == 0)
	{
		Valiad = false;
		return false;
	}

	CaculatePatchCurvetures(positions);

	DirectX::CreateBoundingOrientedBoxFromPoints(BoundingBox, positions.size(), (XMFLOAT3*)positions.data(), sizeof(XMVECTOR));

	CaculatePrinciplePatchOrientation();

	SortOrientedBoundingBox(BoundingBox);

	Valiad = BoundingBox.Extents.z * 2.0f < ZTolerance;
	XMVECTOR q = XMLoad(BoundingBox.Orientation);
	XMVECTOR opticsAxis = XMVector3Rotate(g_XMIdentityR2.v, q);

	bool flipZ = XMVector3Greater(XMVector3Dot(opticsAxis, m_averageNormal), XMVectorZero());
	if (flipZ)
	{
		opticsAxis = -opticsAxis;
		q = XMQuaternionMultiply(XMQuaternionRotationYaw(XM_PI), q);
	}

	XMVECTOR Far = BoundingBox.Center + opticsAxis * BoundingBox.Extents.z;
	XMVECTOR Origin = Far - opticsAxis * m_defaultCameraFrustum.Far;

	m_defaultCameraFrustum.Transform(m_cameraFrustum, XMMatrixRigidTransform(q, Origin));
	return Valiad;
}

void InspectionPatch::CaculateVerticsInPatch(int fid, const DirectX::BoundingOrientedBox &patchRange, XMVECTOR_ARRAY&positions)
{
	using namespace DirectX::VertexTraits;
	XMVECTOR dsize = DecalSize;
	XMVECTOR ext = m_uvExtent;
	XMVECTOR uvmin = m_uvCenter - m_uvExtent;
	XMVECTOR uvmax = m_uvCenter + m_uvExtent;
	auto& vertices = this->m_surface->vertices;

	positions.clear();
	m_areaVertices.clear();
	m_areaFacets.clear();
	//std::cout << "Testing Facets ";
	// m_areaFacets.reserve(); C * PatchArea / SurfaceArea * SurfaceTriangleCount
	m_surface->find_adjacant_facets_of(fid, [this, uvmin, uvmax](tri_idx_t facet_id) -> tri_containment_t {
		tri_containment_t contains;
		//std::cout << facet_id << ',';

		auto& surface = *this->m_surface;
		int count = 0;
		for (int i = 0; i < 3; i++)
		{
			auto& v = surface.vertex(facet_id, i);
			XMVECTOR uv = get_uv(v);
			int inside = XMVector2Greater(uv, uvmin) && XMVector2Less(uv, uvmax);
			count += inside;
			contains.vertex_containments[i] = inside ? DirectX::ContainmentType::CONTAINS : DirectX::ContainmentType::DISJOINT;
		}
		contains.containment = count == 0 ? DirectX::ContainmentType::DISJOINT :
			count == 3 ? DirectX::ContainmentType::CONTAINS :
			DirectX::ContainmentType::INTERSECTS;
		return contains;
	}, m_areaFacets);

	//std::cout << "=============== Testing end =============" << std::endl;

	m_areaVertices.clear();
	for (auto& fc : m_areaFacets)
	{
		int fid = fc.first;
		auto& vin = fc.second.vertex_containments;
		for (int i = 0; i < 3; i++)
			m_areaVertices[surface().indices[fid * 3 + i]] = fc.second.vertex_containments[i];

		if (fc.second.containment == DirectX::ContainmentType::INTERSECTS)
		{
			for (int e = 0; e < 3; e++)
			{
				if (vin[e] ^ vin[(e + 1) % 3])
				{
					auto v0 = surface().vertex(fid, e);
					auto v1 = surface().vertex(fid, (e + 1) % 3);
					AddCrossEdgeIntersections(v0, v1, positions);
				}
			}
		}
	}

	XMVECTOR avNorm = XMVectorZero();
	for (auto& vc : m_areaVertices)
	{
		int vid = vc.first;
		if (vc.second != DirectX::ContainmentType::DISJOINT)
		{
			auto& v = vertices[vid];
			XMVECTOR norm = get_normal(v);
			avNorm += norm;
			positions.push_back(get_position(v));
		}
	}
	m_averageNormal = DirectX::XMVector3Normalize(avNorm);
}

void InspectionPatch::CaculatePatchCurvetures(XMVECTOR_ARRAY &positions)
{
	auto& vertices = m_surface->vertices;
	m_curvetures.clear();
	for (auto& fc : m_areaFacets)
	{
		auto& facet = surface().facet(fc.first);
		auto& is_vertex_in_area = fc.second.vertex_containments;
		for (int e = 0; e < 3; e++)
		{
			int iv0 = facet[e];
			int iv1 = facet[(e + 1) % 3];

			auto v0_in = is_vertex_in_area[e];
			auto v1_in = is_vertex_in_area[(e + 1) % 3];
			if (v0_in || v1_in)
			{
				auto v0 = vertices[iv0];
				auto v1 = vertices[iv1];

				XMVECTOR ip = GetEdgeCurveture(v0, v1);
				m_curvetures.emplace_back(ip);
			}
		}
	}
}

void InspectionPatch::AddCrossEdgeIntersections(const DirectX::VertexPositionNormalTangentColorTexture &v0, const DirectX::VertexPositionNormalTangentColorTexture &v1, std::vector<DirectX::XMVECTOR, DirectX::XMAllocator> & positions)
{
	XMVECTOR uv0 = get_uv(v0);
	XMVECTOR uv1 = get_uv(v1);
	XMVECTOR b1;
	XMVECTOR e1 = m_uvCurve[0];
	XMVECTOR uvi;
	for (int i = 1; i < m_uvCurve.size(); i++)
	{
		b1 = e1;
		e1 = m_uvCurve[i];

		XMVECTOR t = DirectX::LineSegmentTest::RayIntersects2D(uv0, uv1 - uv0, b1, e1);
		if (!DirectX::XMVector4IsNaN(t) && DirectX::XMVector4Less(t, g_XMOne.v))
		{
			//add a new persudo vertex in
			uvi = DirectX::XMVectorLerpV(uv0, uv1, t);
			t = DirectX::XMVectorLerpV(get_position(v0), get_position(v1), t);
			positions.push_back(t);
		}
	}

}

XMVECTOR XM_CALLCONV InspectionPatch::GetEdgeCurveture(const DirectX::VertexPositionNormalTangentColorTexture &v0, const DirectX::VertexPositionNormalTangentColorTexture &v1)
{
	XMVECTOR p0 = get_position(v0);
	XMVECTOR n0 = get_normal(v0);
	XMVECTOR p1 = get_position(v1);
	XMVECTOR n1 = get_normal(v1);

	XMVECTOR edir = p1 - p0;

	ProjectEdgeInTangentNormalPlane(p1, p0, n0, n1);

	// No do the intersection in 2D
	XMVECTOR ip = XMVector2IntersectLine(p0, p0 + n0, p1, p1 + n1);
	// Now caculate the curvture base on intersection distantce
	if (XMVector4IsNaN(ip)) // paradelle to each other, 0 curvture
		ip = XMVectorZero();
	else
	{
		// Caculate the average cruvture circle radius
		XMVECTOR c0 = ip + ip - p0 - p1;
		XMVECTOR c = XMVector3Length(c0) * 0.5f; // In fact *.5 is useless as we donot need the curvature up to a scale
		if (XMVector4Less(XMVector3Dot(c0, n0), XMVectorZero()))
			c = -c;
		ip = XMVectorReciprocal(c);
	}

	//XMVECTOR uv0, uv1;
	//uv0 = get_uv(v0);
	//uv1 = get_uv(v1);
	//uv1 = uv1 - uv0;

	//uv1 = XMVector3Rotate(uv1, XMQuaternionConjugate(patchOrientation));

	//uv0 = XMVectorSplatY(uv1);
	//uv1 = XMVectorSplatX(uv1);
	//XMVECTOR phi = XMVectorATanEst(uv0 / uv1);
	// X = phi, Y = curveture

	// stores the edge direction and the curvture among it
	ip = _DXMEXT XMVectorSelect<0, 0, 0, 1>(edir, ip);

	return ip;
}

void InspectionPatch::ProjectEdgeInTangentNormalPlane(DirectX::XMVECTOR &p1, DirectX::XMVECTOR &p0, DirectX::XMVECTOR &n0, DirectX::XMVECTOR &n1)
{
	// project p0,p1,n0,n1 into intrinsic coordinate, ignores the bi-normal direction
	XMMATRIX rot;
	XMVECTOR axisX = _DXMEXT XMVector3Normalize(p1 - p0);
	XMVECTOR axisY = XMVectorAdd(n0, n1);
	// idely, the normal should be perpendicular to the edge
	// but it may not be true in the real world case
	// use two Cross-Product to build a orthgnal base
	XMVECTOR axisZ = XMVector3Cross(axisX, axisY);
	axisY = XMVector3Cross(axisZ, axisX);
	axisZ = XMVector3Cross(axisX, axisY);
	// Build the Rotation-Projection Matrix, conceptually:
	// rot.r[0] = axisX; rot.r[1] = axisY; rot.r[2] = axisZ; rot.r[3] = g_XMZero;
	// Notice rot is row-major, which means rot' == rotation from intrinsic to original
	// And that rot == rot'' == inv(rotation from intrinsic to original)
	// thus rot == rotation original from to intrinsic
	// And we wish to ingnore the Z/W coordinate, so that we set rot = rot * dialog(1,1,0,0)
	// Which means set Z components of axisX/Y/Z to zero
	XMVECTOR mask = g_XMSelect1100.v;
	axisX = XMVectorAndInt(axisX, mask); axisY = XMVectorAndInt(axisY, mask); axisZ = XMVectorAndInt(axisZ, mask);
	rot.r[0] = axisX; rot.r[1] = axisY; rot.r[2] = axisZ; rot.r[3] = g_XMZero;

	// No Translate, ingnore W part
	p0 = XMVector3TransformNormal(p0, rot);
	p1 = XMVector3TransformNormal(p1, rot);
	n0 = XMVector3TransformNormal(n0, rot);
	n1 = XMVector3TransformNormal(n1, rot);
}

void XM_CALLCONV InspectionPatch::CaculatePrinciplePatchOrientation()
{
	XMVECTOR orientation = XMLoad(BoundingBox.Orientation);
	XMVECTOR projQuat = XMQuaternionConjugate(orientation);

	// convert the edge into principle coords
	// and ignores its Z componet
	// and get the 'angle' in this 2D coords
	auto proj = [projQuat](FXMVECTOR v) -> float
	{
		XMVECTOR V = XMVector3Rotate(v, projQuat);
		float a = atan2f(XMVectorGetY(V), XMVectorGetX(V));
		a = fmodf(a + XM_PI, XM_PI); // we need the result in [0,pi)
		return a;
	};

	std::vector<Vector2> cuv_phi; cuv_phi.reserve((m_curvetures.size()));
	for (auto& c : m_curvetures)
		cuv_phi.emplace_back(proj(c), /*XMVectorGetX(XMVector3Length(c))**/c.w);

	// angle resolution of our minimumlizer
	// since this function is __NOT__ CONVEX at all, use search to do this
	CurvtureDistribution.resize(CurvtureAngluarResolution);
	auto f = [&cuv_phi](float theta)
	{
		float val = 0;
		for (auto& c : cuv_phi)
		{
			float dis = fabsf(theta - c.x);
			dis = fminf(dis, fabsf(XM_PI - dis));
			float fallout = XMScalarCosEst(dis);
			fallout *= fallout; // cos may not be localized enough
			//fallout *= fallout; // use cos^4 instead ! (any bell curv may be fine)
			val += fallout * fabsf(c.y); // we donnot need a high precision cos
		}
		return val;
	};
	auto delta = XM_PI / CurvtureAngluarResolution;
	int minid = 9; float minval = f(0);
	CurvtureDistribution[0] = minval;
	for (int i = 1; i < CurvtureAngluarResolution; i++)
	{
		auto val = f(i*delta);
		CurvtureDistribution[i] = val;
		if (val < minval) { minval = val; minid = i; }
	}

	// normalize it
	float maxval = *std::max_element(CurvtureDistribution.begin(), CurvtureDistribution.end());
	for (auto& c : CurvtureDistribution)
		c /= maxval;

	m_principleUvRotation = minid * delta;

	(Quaternion&)BoundingBox.Orientation = XMQuaternionMultiply(XMQuaternionRotationRoll(m_principleUvRotation), orientation);
}

void XM_CALLCONV InspectionPatch::SetSurfaceCurveToUVRect(FXMVECTOR uvc, FXMVECTOR uvext)
{
	m_uvCenter = uvc;
	m_uvExtent = uvext;
	m_uvRotation = .0f;

	UpdateDecalTransformMatrix();

	XMVECTOR onx = uvext;
	XMVECTOR flx = g_XMNegateY.v * uvext;

	m_uvCurve.clear();
	m_uvCurve.push_back(uvc - onx, true); // (-1,-1)
	m_uvCurve.push_back(uvc + flx, true); // ( 1,-1)
	m_uvCurve.push_back(uvc + onx, true); // ( 1, 1)
	m_uvCurve.push_back(uvc - flx, true); // (-1, 1)
	m_uvCurve.closeLoop();
	m_dirty = 3;
}

inline float2 XM_CALLCONV uv2xy(float2 uv)
{
	// [0,1;0,1] -> [0,2;-2,0] -> [-1,1;-1;1]
	return uv * float2(2, -2) + float2(-1, 1);
}

inline float2 XM_CALLCONV xy2uv(float2 xy)
{
	return (xy + float2(1, 1)) * float2(0.5f, -0.5f);
}

void InspectionPatch::UpdateGeometry(I2DFactory* pFactory, bool smooth)
{
	auto& points = this->m_uvCurve.anchors();
	auto smoothed = m_uvCurve.sample(100);

	if (points.size() <= 1)
		return;

	pFactory->CreatePathGeometry(&m_deaclGeometry);
	cptr<ID2D1GeometrySink> pSink;
	ThrowIfFailed(m_deaclGeometry->Open(&pSink));
	pSink->SetFillMode(D2D1_FILL_MODE_WINDING);

	float2 scl = load(DecalSize);

	int n = smooth ? smoothed.size() : points.size();
	n -= !smooth; // anchors are more sized
	vector<D2D1_POINT_2F> dpoints(n);

	for (int i = 0; i < n; i++)
	{
		float2 v;
		//if (smooth)
		//	v = load(smoothed[i]).xy();
		//else
		v = load(points[i]).xy();

		//std::cout << v << "=>";
		v = v * scl;
		//std::cout << v << std::endl;

		dpoints[i] << v;
	}

	pSink->BeginFigure(
		dpoints[0],
		D2D1_FIGURE_BEGIN_FILLED
	);
	pSink->AddLines(&dpoints[1], n - 1);
	pSink->EndFigure(D2D1_FIGURE_END_CLOSED);
	pSink->Close();
	pSink.Reset();

	if (CurvtureDistribution.empty()) return;

	pFactory->CreatePathGeometry(&m_curvHistoGeometry);
	ThrowIfFailed(m_curvHistoGeometry->Open(&pSink));
	pSink->SetFillMode(D2D1_FILL_MODE_WINDING);
	vector<D2D1_POINT_2F> chisto(CurvtureAngluarResolution * 2);

	const float figureRadius = 0.25f;
	const float2 figureCenter = { 0,75,0.75 };
	for (int i = 0; i < CurvtureAngluarResolution * 2; i++)
	{
		float a = i* XM_PI / CurvtureAngluarResolution;
		float m = figureRadius * CurvtureDistribution[i % CurvtureAngluarResolution];
		float sina, cosa;
		XMScalarSinCosEst(&sina, &cosa, a);
		float2 v = { cosa, sina };
		v = v * m + figureCenter;
		v = v * m_uvExtent + m_uvCenter;
		v *= scl;
		chisto[i] << v;
	}
	pSink->BeginFigure(
		chisto[0],
		D2D1_FIGURE_BEGIN_FILLED
	);
	pSink->AddLines(&chisto[1], chisto.size() - 1);
	pSink->EndFigure(D2D1_FIGURE_END_CLOSED);

	pSink->Close();
	pSink.Reset();

	m_dirty = 1;
}

void InspectionPatch::DrawDecal(I2DContext * pContext, ID2D1SolidColorBrush * pBrush)
{
	Color color = Colors::Red.v;
	if (Valiad)
		color = Colors::Lime.v;
	color.w = m_opticity;

	UpdateDecalTransformMatrix();

	// S
	auto trans = D2D1::Matrix3x2F::Identity();
	pContext->SetTransform(trans);

	pBrush->SetColor(GetD2DColor(color));
	pContext->FillGeometry(m_deaclGeometry.Get(), pBrush);

	color = Colors::Black.v;
	pBrush->SetColor(GetD2DColor(color));
	pContext->DrawGeometry(m_curvHistoGeometry.Get(), pBrush, 2.0f / DecalSize.Length());
	color = Colors::White.v;
	pBrush->SetColor(GetD2DColor(color));
	if (m_curvHistoGeometry)
		pContext->FillGeometry(m_curvHistoGeometry.Get(), pBrush);
}

void InspectionPatch::UpdateDecalTransformMatrix()
{
	// S
	m_decalTransform = D2D1::Matrix3x2F::Scale(m_uvExtent.x, m_uvExtent.y);
	// R
	m_decalTransform = m_decalTransform * D2D1::Matrix3x2F::Rotation(m_uvRotation);
	// T
	m_decalTransform = m_decalTransform * D2D1::Matrix3x2F::Translation(m_uvCenter.x, m_uvCenter.y);
}
