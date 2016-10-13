#pragma once
#include <DirectXMath.h>
#include "traits.hpp"

#define _HLSL_TRAITS_NAMESPACE ::DirectX::hlsl::traits

#define _HLSL_REGISTER_VECTOR_TYPE_(_VectorType_, _Scalar_, _Rows_, _Cols_) \
template <> struct _HLSL_TRAITS_NAMESPACE::vector_traits<_VectorType_> { \
	static constexpr int rows = _Rows_; \
	static constexpr int cols = _Cols_; \
	using scalar = _Scalar_; }

namespace DirectX
{
	namespace hlsl
	{
		namespace traits
		{
			template <typename _Ty, size_t sz>
			struct vector_traits<_Ty[sz]>
			{
				static constexpr int rows = 1;
				static constexpr int cols = sz;
				using scalar = _Ty;
			};

			template <typename _Ty, size_t sz_row, size_t sz_col>
			struct vector_traits<_Ty[sz_row][sz_col]>
			{
				static constexpr int rows = sz_row;
				static constexpr int cols = sz_col;
				using scalar = _Ty;
			};

			template <>
			struct vector_traits<uint>
			{
				static constexpr int rows = 1;
				static constexpr int cols = 1;
				using scalar = uint;
			};

			template <>
			struct vector_traits<float>
			{
				static constexpr int rows = 1;
				static constexpr int cols = 1;
				using scalar = float;
			};
		}
	}
}

// REGISTER DirectXMath Storage Types
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT2, float, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT3, float, 1, 3);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT4, float, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT2A, float, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT3A, float, 1, 3);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMFLOAT4A, float, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMVECTORF32, float, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMVECTORU32, uint32_t, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMVECTORI32, int32_t, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(XM_NAMES XMVECTORU8, uint8_t, 1, 16);

// Direct2D Types
#ifdef _D2DBASETYPES_INCLUDED
_HLSL_REGISTER_VECTOR_TYPE_(D2D_POINT_2F, float, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(D2D_POINT_2U, uint32_t, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(D2D_COLOR_F, float, 1, 4);
_HLSL_REGISTER_VECTOR_TYPE_(D2D_SIZE_F, float, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(D2D_SIZE_U, uint32_t, 1, 2);
_HLSL_REGISTER_VECTOR_TYPE_(D2D_MATRIX_3X2_F, float, 3, 2);
#endif // _D2DBASETYPES_INCLUDED

