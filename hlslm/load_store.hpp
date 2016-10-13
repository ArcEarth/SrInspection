#pragma once
#ifndef _HLSL_XM_LOAD_STORE_H
#define _HLSL_XM_LOAD_STORE_H
#ifndef _HLSL_XM_VECTOR_H
#include "xmvector.hpp"
#endif

#include <type_traits>
#include "storage_classes.hpp"

namespace DirectX
{
namespace hlsl
{
	namespace traits
	{
		// the corresponding xmvector type of a memory vector
		template <typename _Ty>
		using if_memory_vector_type = typename traits::enable_memery_traits_t<_Ty>::type;
	}

	template <typename _Ty>
	inline traits::if_memory_vector_type<_Ty> load(const _Ty& memory_vector)
	{
		using traits = traits::memery_vector_traits<_Ty>;
		using load_imple = detail::storage_helper<typename traits::scalar, ::hlsl::traits::is_aligned<_Ty>::value, traits::cols, traits::rows>;
		typename traits::type ret;
		ret.v = load_imple::load(reinterpret_cast<const typename traits::scalar*>(&memory_vector));
		return ret;
	}

	template <typename _Ty>
	inline traits::if_memory_vector_type<_Ty> load_a(const _Ty& memory_vector)
	{
		using traits = traits::memery_vector_traits<_Ty>;
		using load_imple = detail::storage_helper<typename traits::scalar, true, traits::cols, traits::rows>;
		typename traits::type ret;
		ret.v = load_imple::load(reinterpret_cast<const typename traits::scalar*>(&memory_vector));
		return ret;
	}

	template <typename _Ty>
	inline void XM_CALLCONV store(_Ty& memory_vector, const traits::if_memory_vector_type<_Ty> v)
	{
		using traits = traits::memery_vector_traits<_Ty>;
		using load_imple = detail::storage_helper<typename traits::scalar, ::hlsl::traits::is_aligned<_Ty>::value, traits::cols, traits::rows>;
		load_imple::store(reinterpret_cast<typename traits::scalar*>(&memory_vector),v.v);
	}

	// store stream operator <<
	template <typename _Ty>
	inline _Ty& XM_CALLCONV operator<<(_Ty& memory_vector, const traits::if_memory_vector_type<_Ty> v)
	{
		store<_Ty>(memory_vector, v);
		return memory_vector;
	}

	template <typename _Ty>
	inline void XM_CALLCONV store_a(_Ty& memory_vector, const traits::if_memory_vector_type<_Ty> v)
	{
		using traits = typename traits::memery_vector_traits<_Ty>;
		using load_imple = typename detail::storage_helper<typename traits::scalar, true, traits::cols, traits::rows>;
		load_imple::store(reinterpret_cast<typename traits::scalar*>(&memory_vector),v.v);
	}

	template <typename _Ty, bool _is_aligned = false>
	struct memery_vector_wrapper
	{
		static constexpr bool aligned = traits::is_aligned<_Ty>::value || _is_aligned;
		static_assert(traits::is_memory_type<_Ty>::value, "_Ty must be a registered memery vector type");
		using traits = typename traits::memery_vector_traits<_Ty>;
		using xmvector_type = typename traits::type;
		using ls_impl = typename detail::storage_helper<typename traits::scalar, aligned, traits::cols, traits::rows>;

		_Ty data;
		memery_vector_wrapper() = delete;
		~memery_vector_wrapper() = delete;

		memery_vector_wrapper& XM_CALLCONV operator=(const xmvector_type v)
		{
			ls_impl::store(reinterpret_cast<typename traits::scalar*>(this), v.v);
			return *this;
		}

		operator xmvector_type() const
		{
			xmvector_type ret;
			ret.v = ls_impl::load(reinterpret_cast<const typename traits::scalar*>(this));
			return ret;
		}
	};

	// helper function xm
	template <typename _Ty>
	memery_vector_wrapper<_Ty>& xm(_Ty& memory_vector) {
		return reinterpret_cast<memery_vector_wrapper<_Ty>&>(memory_vector);
	}
	template <typename _Ty>
	memery_vector_wrapper<_Ty>& xm(const _Ty& memory_vector) {
		return reinterpret_cast<const memery_vector_wrapper<_Ty>&>(memory_vector);
	}



	template <typename _Scalar>
	inline std::enable_if_t<traits::scalar_traits<_Scalar>::value, xmscalar<_Scalar>> XM_CALLCONV load(_Scalar scalar)
	{ return xmscalar<_Scalar>(detail::replicate_scalar(scalar)); }
	template <typename _Ty, index_t... _SwzArgs>
	typename xmvector<_Ty, sizeof...(_SwzArgs)> XM_CALLCONV load(const xmswizzler<_Ty, _SwzArgs...>& swzizzler)
	{ return swzizzler.eval(); }
	template <typename _Ty, index_t... _SwzArgs>
	typename void XM_CALLCONV store(xmswizzler<_Ty, _SwzArgs...>& swzizzler, const xmvector<_Ty, sizeof...(_SwzArgs)> v)
	{ swzizzler.assign(v); }

	// load function pass by 
	template <typename _VectorType>
	inline std::enable_if_t<traits::is_xmvector<_VectorType>::value, _VectorType&> XM_CALLCONV load(_VectorType& v) { return v; }

	template <typename _VectorType>
	inline std::enable_if_t<traits::is_xmvector<_VectorType>::value, _VectorType&&> XM_CALLCONV load(_VectorType&& v) { return v; }

	template <typename _VectorType>
	inline std::enable_if_t<traits::is_xmvector<_VectorType>::value, const _VectorType&> XM_CALLCONV load(const _VectorType& v) { return v; }


}
}
#endif