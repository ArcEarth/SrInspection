#include <chrono>
#include <String>
#include <vector>

namespace Causality::SurfaceInspection
{
	namespace Units
	{
		// Standard for length
		// Intend for cross function / module communacation
		template <int _Num, int _Den>
		struct length_t
		{
			using conv = std::ratio<_Num, _Den>;
			double val;
			explicit length_t(double _val) : val(_val) {}
			template <int _ONum, int _ODen>
			operator length_t<_ONum, _ODen>() const { return length_t<_ONum, _ODen>(val * _Num / _Den * _ODen / _ONum); }
		};
		using meter_t = length_t<1, 1>;
		using decimeter_t = length_t<1, 10>;
		using centimeter_t = length_t<1, 100>;
		using millimeter_t = length_t<1, 1000>;
		using micrometer_t = length_t<1, 1000000>;

		meter_t operator "" _m(long double val) { return meter_t(val); }
		decimeter_t operator "" _dm(long double val) { return decimeter_t(val); }
		centimeter_t operator "" _cm(long double val) { return centimeter_t(val); }
		millimeter_t operator "" _mm(long double val) { return millimeter_t(val); }
		micrometer_t operator "" _um(long double val) { return micrometer_t(val); }

		// Standard for time
		using time_span = std::chrono::duration<double>;
		using time_point = std::chrono::time_point<std::chrono::steady_clock, time_duration>;
		using namespace std::chrono_literals;
	}

	// A matrix is a column major matrix
	// _dataptr has strong-ownership
	template <typename _Ty>
	struct Matrix
	{
		using scalar_t = _Ty;
		scalar_t* _dataptr;
		size_t	  _rows;
		size_t	  _cols;
		size_t	  _col_stride;

		size_t rows() const { return _rows; }
		size_t cols() const { return _cols; }
	};

	template <typename _Ty, int Rows, int Cols>
	struct MatrixFixed
	{
		using scalar_t = _Ty;
		scalar_t _dataptr[Rows*Cols];
		static constexpr size_t rows() { return Rows; }
		static constexpr size_t cols() { return Cols; }
	};

	using MatrixS = Matrix<int16_t>;
	using MatrixI = Matrix<int32_t>;
	using MatrixL = Matrix<int64_t>;
	using MatrixF = Matrix<float>;
	using MatrixD = Matrix<double>;
	using String  = std::string;

	enum ColorMode {
		ColorMode_Monochrom = 0,
		ColorMode_RGB = 1,
	};

	struct RawImage {
		MatrixF   Data; // Raw Data before demossica
		ColorMode ColorMode;
		int		  DecodeHeight;
		int		  DecodeWidth;
	};

	// standard in second
	using TimeSpan = Units::time_span;
	using TimePoint = Units::time_point;
	// standard in meter
	using Length = Units::meter_t;
	using ErrorCode = int;

    struct ExplosureState
    {
		TimeSpan	 ExplosureTime;
        float		 Aperture;
        float		 ISO;
        float		 Fucus;
    };
    
    struct FlushState
    {
        bool    UseFlush;
        int     LightPattern;
        float   FluxPerPixel;
        float   Focus;
    };
    
    struct CameraCaptureResult
    {
        ErrorCode      Error;
        TimePoint      TimeStemp;
        ExplosureState ActualExplosure;
        FlushState     ActualFlush;
        RawImage       Image;
    };
    
    // Specification about the 'Camera-Projector' system
    class ICameraProjectorSystem
    {
		String      Name;
		String      Address;

        long long   Version;
		Length      DepthCameraFocus;       // in milimeter
        Length      ProjectorFocus;         // in milimeter
        float       DepthCameraAperture;
        vector<float> DepthCameraISORange;  // ISO ranks
        float       ProjectorFluxPerPixel;  // Unit in Watts/pixel
        float       DepthResolution;        // in milimeter
        Vector2i    DepthCameraResolution;  // in pixels
        Vector2i    ProjectorResolution;    // in pixels
        RigidTransform ProjectorDispacement; // Physical displacement for the projector, From camera coordinate, unit in milimeter
        Matrix4f    DepthCameraProjection;
        Matrix4f    ProjectorProjection;
        
        virtual bool                IsConnected() const = 0;
        virtual future<ErrorCode>   DisconnectAsync(); 
        virtual future<ErrorCode>   ConnectAsync(String name, String address) = 0;
        
        // This method should synchronze the projector and camera so that we can capture a desired patch with given flush 
        virtual future<CameraCaptureResult>    CaptureAsync(ExplosureState explosure_params, FlushState flush_params) = 0;
        
        // Projector ViewProjection = ProjectionDisplacement^-1 * Projector ProjectionDisplacement
        
        // More information about the camera-projector system
    };

    struct EffectorState
    {
        TimePoint   ReportTime;
        bool        IsStable;
        Vector3f    Position;
        Quaternion  Orientation;
        Vector3f    PositionError;      
        Quaternion  OrientationError; // Not sure
        Vector3f    TranslationVelocity;      
        Vector3f    AngularVelocity; // Not sure
    };


    class IRoboticArm
    {
        struct PlannedMovement
        {
            ErrorCode           Error;
            TimeSpan            EstimateTimeCost;
            future<ErrorCode>   Done; // Async data signals when finished
        };
                
        virtual String          GetName() const = 0;
        virtual String          GetAddress() const = 0;

        virtual bool            IsConnected() const = 0;

        virtual future<ErrorCode> DisconnectAsync() = 0; 
        virtual future<ErrorCode> ConnectAsync(String name, String address) = 0;
        
        virtual EffectorState   GetEffectorDisplacement() const = 0;
        
        virtual PlannedMovement MoveEffectorTo(Vector3f position, Quaternion orientation) = 0;
    };
    
    // A workspace is a setup with of the 'camera-projector system' mounted on a 'robotic arm' and a table 'workspace'
    class IWorkspaceProfile : public ISearializable
    {
        int     UID; 
        String  Name; // Get Set

        String  RoboticArmName; // Get Set
        String  RoboticArmAddress; // Get Set

        int     CameraArmCalibrationPhase; // -1 signals not calibrated at all
        int     WorkspaceArmCalibrationPhase; // -1 signals not calibrated at all

        RigidTransform WorkspaceInArmCoordinate;
        RigidTransform CameraInArmCoordinate;

        // Set the workspace coordinate in the robotic arm system
        void SetupWorkspaceInArm(const RigidTransform& w_ac);

        // interface needed for interactive calibration
        future<ErrorCode> CalibrateWorkspaceInArm(int phase);
        future<ErrorCode> CalibrateCameraInArm(int phase);
    };
    
    struct InspectionPatch
    {
        RigidTransform CameraDisplacement; // From the model, thus to caculate the final robotic camera's displacement, you must combines it with a WorkspaceProfile 
    };
    
    class IPartInspectionPlan : public ISearializable
    {
        int                         UID;
        String                      Name;
        
        String                      PartName;
        String                      PartMeshFilePath;
        
        virtual future<ErrorCode>   LoadPartMesh() = 0;
        virtual IInspectionMesh*    GetPartMesh() const = 0;

        //! This is a PLANNED part displacement from the 'Workspace Coordinate'
        //! When applying an actual inspection task, the part's ACTUAL displacement must be re-calibrated
        RigidTransform              PlannedPartDisplacement; // Get Set
        
        // The planned camera displacement in the part's model file's local coordinate 
        vector<InspectionPatch>     InspectionPath;
    };
    
    enum InspectionState
    {
        Error = -1,
        Initialzing,
        PartLocalizing,
        PatchesCapturing,
        Finalizing,
        Done,
    };
        
    class DepthPatchCaptureResult : public CameraCaptureResult
    {
        EffectorState   CameraLocation; 
        MatrixF         DepthMap;   // The Caculated Depth map from the W/B image;
    };
    
    class SurfaceDefectPatch : public SurfacePatch
    {
        enum DeflectionType
        {
            Deflection_Dent, // A point-like deflection 
            Deflection_Stratch, // A linear stractch deflection
            Deflection_Unknown,
        };
    };
    
    class IPartLocalizer {
        
    };

	class IInspectionMesh {

	};

    class IDepthReconstructor {
		virtual vector<> GetRequestCameraProjectorSetup() = 0;
        virtual MatrixF CaculateDepthMap(const CameraCaptureResult& image, ICameraProjectorSystem* system_metric) = 0;
    };
    
    class IPatchFuser {
        // the displacement map is a scalar field of the HEIGHT displacement from the actual surface to the base surface
        // in the direction of SURFACE NORMAL 
        // value QNaN signals that pixel is invaliad
        virtual MatrixXf CaculateFusedDisplacementMap(const IInspectionMesh* part_mesh, const vector<DepthPatchCaptureResult*>& patches) = 0;
    };
    
    class IDefectDectector{
        vector<DepthPatchCaptureResult> DetectDefects(const IInspectionMesh* part_mesh, const MatrixXf& displacement_map);
    };
    
    // a Instance of the actual inspection to a specific part on a specific workspace
    class IInspectionTask
    {
        ICameraProjectorSystem* Camera;
        IRoboticArm*            Arm;
        IWorkspaceProfile*      Workspace;
        IInspectionMesh*        PartMesh;
        IPartInspectionPlan*    Plan;
        
        IPartLocalizer*         PartLocalizer;
        IDepthReconstructor*    DepthReconstructor;
        IPatchFus*				PatchFuser;
        IDefectDectector*       DefectDetector;
        
        virtual InspectionState CurrentState() const = 0;
        virtual future<ErrorCode> Run() = 0;

        virtual RigidTransform  GetActualPartDisplacement() const = 0;

        virtual int             PatchesInspected() const = 0;
        DepthPatchCaptureResult*GetInspectedPatch(int pitch_id) const = 0;
        
        // Fuser 
        virtual const MatrixXf& GetFusedDisplacementMap() const = 0;

        // Defect Detector Result
        virtual int             GetDefectCount() const = 0;
        SurfaceDefectPatch      GetDefectPatch(int defect_id) const = 0;
        virtual IInspectionMesh*GetDeformedMesh() const = 0;
        virtual bool            IsMeshGlobalDeformedOverLimit() const = 0;
    };
    
    // class IInspectionPlanner
    // {
    //     virtual future<unique_ptr<IPartInspectionPlan>> CreatePlan() = 0;
    // };

}