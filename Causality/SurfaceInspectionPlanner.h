#pragma once
#include <Causality\VisualObject.h>
#include <Geometrics\BezierMesh.h>
#include <VertexTypes.h>
#include <atomic>
#include <Textures.h>
#include <Geometrics\Extrusion.h>

namespace Causality
{
	typedef Geometrics::Bezier::BezierPatch<Vector3, 3U> CubicBezierPatch;
	typedef CubicBezierPatch::ClippingType CubicBezierCurve;
	class IPointer;
	class TrackedPen;

	namespace SurfaceInspection
	{
		using TriangleMeshType = Geometrics::TriangleMesh<DirectX::VertexPositionNormalTangentColorTexture, uint32_t>;

		using MeshType = TriangleMeshType;

		struct InspectionCameraParameter
		{
			float FieldOfView; // In Rad, X-axis
			float Aspect;	   // H/W
			float Focus;	   // Focusd Distance
			float DepthOfField;// Z-Tolerence
		};

		struct InspectionPatch : public Geometrics::SurfacePatch<MeshType>
		{
			static InspectionCameraParameter DefaultCameraParam;
			static float			DefaultUvScale;		// 50

			using base_type = Geometrics::SurfacePatch<MeshType>;
			InspectionPatch();

			void SetCameraParameter(float fov_x_rad, float aspect_h_by_w, float focus_distance, float focus_dof, float uvscale);

			Vector2					DecalSize;

			bool					m_isComplex;
			Vector2					m_uvCenter;
			Vector2					m_uvExtent;

			MeshType::IntersectionVertex
									m_intersectedVertex;
			Vector3					m_center;
			Vector3					m_centerNormal;
			Vector3					m_extent;

			Vector2					m_uvTranslation;
			Vector2					m_uvScaling;
			float					m_uvRotation;
			float					m_principleUvRotation;
			Vector3					m_averageNormal;
			Vector3					m_averageTangent;
			Vector3					m_averageBinormal;

			float					Area;
			float					ZTolerance;
			BoundingOrientedBox		BoundingBox;

			static constexpr int	CurvtureAngluarResolution = 64U;
			vector<float>			CurvtureDistribution;

			bool					Valiad;
			BoundingFrustum			m_cameraFrustum;
			BoundingFrustum			m_defaultCameraFrustum;

			D2D1::Matrix3x2F		m_decalTransform;
			bool					m_requireRedraw;

			Color					m_fillColor;
			Color					m_borderColor;
			float					m_borderThinkness;
			float					m_opticity;

			cptr<ID2D1PathGeometry> m_deaclGeometry;
			cptr<ID2D1PathGeometry> m_curvHistoGeometry;

			using tri_idx_t = TriangleMeshType::IndexType;
			using tri_containment_t = Geometrics::PolygonContainmentInfo<3>;

			std::vector<Vector4>	m_curvetures;
			std::unordered_map<tri_idx_t, Geometrics::PolygonContainmentInfo<3>>
									m_areaFacets;
			std::unordered_map<tri_idx_t, DirectX::ContainmentType>
									m_areaVertices;

			using XMVECTOR_ARRAY = std::vector<DirectX::XMVECTOR, DirectX::XMAllocator>;

			// Principle methods should be called once the patch is update
			bool CaculateCameraFrustum();

			// Helper, caculates the vertices with in the patch
			void CaculateVerticsInPatch(int fid, const DirectX::BoundingOrientedBox &patchRange, XMVECTOR_ARRAY &positions);

			// Helper, caculates the patch's edges curves into array m_curvetures
			void CaculatePatchCurvetures(XMVECTOR_ARRAY &positions);

			void AddCrossEdgeIntersections(const DirectX::VertexPositionNormalTangentColorTexture &v0, const  DirectX::VertexPositionNormalTangentColorTexture &v1, std::vector<DirectX::XMVECTOR, DirectX::XMAllocator> & positions);

			DirectX::XMVECTOR XM_CALLCONV GetEdgeCurveture(const DirectX::VertexPositionNormalTangentColorTexture &v0, const DirectX::VertexPositionNormalTangentColorTexture &v1);

			static void ProjectEdgeInTangentNormalPlane(DirectX::XMVECTOR &p1, DirectX::XMVECTOR &p0, DirectX::XMVECTOR &n0, DirectX::XMVECTOR &n1);

			// Helper, Find the minmimal curveture direction from m_curvestures data
			// The 3D orientation of this patch, where the rotated Z-axis must be the optics axis direction
			void XM_CALLCONV CaculatePrinciplePatchOrientation();

			void XM_CALLCONV SetSurfaceCurveToUVRect(FXMVECTOR uvc, FXMVECTOR uvext);
			void UpdateGeometry(I2DFactory* pFactory, bool smooth = false);
			void DrawDecal(I2DContext* pContext, ID2D1SolidColorBrush* target);
			void UpdateDecalTransformMatrix();
		};

		typedef std::vector<InspectionPatch> InspectionPath;

		class SurfaceSketchManager
		{
		public:
			int Begin(InspectionPatch& patch);
			void End();
			int Update(const MeshType::IntersectionVertex& iv);
		};

		class SurfaceInspectionPlanner : public SceneObject, public IVisual
		{
		public:
		public:
			enum StateEnum
			{
				State_Idle = 0,
				State_DrawingPatch = 1,
				State_DragingPatch = 2,
				State_RotatingPatch = 3,
				State_Erasing = 4,
				State_Initializing = 5,
			};

			void Parse(const ParamArchive* archive) override;

			SurfaceInspectionPlanner();
			~SurfaceInspectionPlanner();
			virtual bool IsVisible(const BoundingGeometry& viewFrustum) const;
			virtual void AddChild(SceneObject* child) override;
			void SetWorkload(Causality::VisualObject * pVO);
			void SetWorkloadFillView(DirectX::Scene::IModelNode * pModel, Causality::VisualObject * pVO);
			virtual void Update(time_seconds const& time_delta) override;
			virtual void Render(IRenderContext * pContext, IEffect* pEffect = nullptr) override;

			void DrawPatchCamera(InspectionPatch& patch);

			virtual RenderFlags GetRenderFlags() const;
			virtual void XM_CALLCONV UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection);;

		private:
			bool XM_CALLCONV HitTest(FXMVECTOR origin, FXMVECTOR dir);

			void DrawBezeirPatchControlPoints();
			void RenderCursor(IRenderContext *pContext, IEffect*pEffect);

			void DrawDecal(I2DContext* pContext);	
			void UpdateDecalGeometry(I2DFactory* pFactory);

			InspectionPatch* TrySelectInspectionPatch(FXMVECTOR uv, int fid = -1);

			// Convert the rendering mesh to computational mesh
			void ExtractWorkloadMesh(const IModelNode* pNode);
			void CreateDecalDeviceResources(DirectX::BoundingBox &bb, DirectX::BoundingOrientedBox &obb);
			void CopyMesh(TriangleMeshType& mesh, const IModelNode* pNode);
			void TesselateBezeirPatch(int tessellation, DirectX::SimpleMath::Color &color);


			InspectionPatch* AddInspectionPatch(const InspectionPatch& patch);
			void CaculateCameraPath();
			void RemovePatch(InspectionPatch* patch);
			void PrintPlan(std::ostream& os) const;
			void GenerateScript() const;

			void SetRectanglePatchPreview(InspectionPatch& patch, const MeshType::IntersectionVertex& iv);

			void SurfaceSketchBegin();
			void SrufaceSketchUpdate(XMVECTOR pos, XMVECTOR dir);
			void SurfaceSketchEnd();

			void SurfacePatchDragBegin(InspectionPatch* patch);
			void SurfacePatchDragUpdate(FXMVECTOR pos);
			void SurfacePatchDragEnd();

			TrackedPen*						m_pen;
			StateEnum						m_state;

			DirectX::Ray					m_castRay;
			bool							m_isHit;
			MeshType::IntersectionVertex	m_intersectedVertex;

			std::vector<int>				m_isCameraPath;
			std::vector<InspectionPatch>	m_isPatches;
			InspectionPatch*				m_activePatch;
			Vector2							m_dragLast;

			CubicBezierPatch				m_bezeirPatch;
			TriangleMeshType				m_mesh;
			VisualObject*					m_workloadObj;

			// Decal texture for rendering highlights in target model
			int								m_declDirtyFalg;
			int								m_requestCancelLoading;

			Vector3							m_projectorDisplacement;
			BoundingFrustum					m_cameraFrustum;
			Matrix4x4						m_cameraProjection;
			InspectionCameraParameter		m_cameraParam;
			float							m_workloadUvScale;

			RenderableTexture2D				m_decal;
			cptr<ID2D1PathGeometry>			m_patchGeos;
			cptr<ID2D1SolidColorBrush>		m_brush;

			Color							m_decalBackground;
			Color							m_decalStroke;
			Color							m_decalFill;

			InspectionPatch					m_previwPatch;
			Vector2							m_dragStart;
			const IPointer*					m_cursor;
			sptr<PhongMaterial>				m_cursorMaterial;

			std::vector<TriangleMeshType>		m_fracorizedMeshes;
			uptr<DirectX::Scene::IModelNode>	m_decalModel;

			uptr<DirectX::Scene::IModelNode>	m_factorizeModel;

			std::atomic_bool				m_isReady;
			concurrency::task<void>			m_loadingTask;
		};
	}
}