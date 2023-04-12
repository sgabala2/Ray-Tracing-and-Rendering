/*
Copyright (c) 2019, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

namespace Util
{
	////////////
	// Matrix //
	////////////
	template< unsigned int Dim >
	Matrix< Dim , Dim > Matrix< Dim , Dim >::Exp( const Matrix &m , int terms )
	{
		//////////////////////////////////////
		// Compute the matrix exponent here //
		//////////////////////////////////////
		WARN_ONCE( "method undefined" );
		return Matrix();
	}

	template< unsigned int Dim >
	Matrix< Dim , Dim > Matrix< Dim , Dim >::closestRotation( void ) const
	{
		///////////////////////////////////////
		// Compute the closest rotation here //
		///////////////////////////////////////
		WARN_ONCE( "method undefined" );
		return Matrix();
	}

	/////////////////
	// BoundingBox //
	/////////////////
	template< unsigned int Dim >
	BoundingBox< 1 > BoundingBox< Dim >::intersect( const Ray< Dim > &ray ) const
	{
		///////////////////////////////////////////////////////////////
		// Compute the intersection of a BoundingBox with a Ray here //
		///////////////////////////////////////////////////////////////
		const Point<Dim> bmin = _p[0], bmax = _p[1];
		Point<Dim> inv_dir;
		for (int i = 0; i < Dim; i++)
			inv_dir[i] = 1 / ray.direction[i];
		double tmin = -Infinity;
		double tmax = Infinity;
		for (int i = 0; i < Dim; i++) {
			const double t1 = (bmin[i] - ray.position[i]) * inv_dir[i];
			const double t2 = (bmax[i] - ray.position[i]) * inv_dir[i];
			tmin = std::max(tmin, std::min(t1, t2));
			tmax = std::min(tmax, std::max(t1, t2));
		}
		auto miss = BoundingBox1D();
		miss[0] = Infinity, miss[1] = -Infinity; 
		return tmax >= tmin && tmax > 0 ? BoundingBox1D(tmin, tmax) : miss;
	}

	/////////////
	// Quadric //
	/////////////
	template< unsigned int Dim >
	Quadric< 1 > Quadric< Dim >::intersect( const Ray< Dim > &ray ) const
	{
		WARN_ONCE( "method undefined" );
		return Quadric<1>();
	}
}