#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/sequence.h>

#include <thrust/transform.h> 
#include <thrust/replace.h> 
#include <thrust/functional.h>
#include <thrust/count.h>
#include <thrust/scan.h>

#include <iostream>
#include <fstream>

#include <thrust/iterator/transform_iterator.h>
#include <thrust/adjacent_difference.h>

#include <queue>

/*
VECTORS
Thrust provides two containters for vector:
- host_vector - stored in CPU
- device_vector - stored in GPU
otherwise, they are exactly like std::vector.
*/
void thrust_vectors_introduction() {

	// INRODUCTION
	thrust::host_vector<int> H(4);
	H[0] = 14; H[1] = 20; H[2] = 38; H[3] = 46;
	for (int i = 0; i < H.size(); i++)
		std::cout << "H[" << i << "] = " << H[i] << " ";
	std::cout << std::endl;

	// Operator "=" is used to copy_host vector to device_vector;
	thrust::device_vector<int> D = H;

	// Each access through operator "[]" requires use of cudaMemcpy, thus use sparingly
	D[0] = 99; D[1] = 88;

	// print contents of D
	for (int i = 0; i < D.size(); i++)
		std::cout << "D[" << i << "] = " << D[i] << " ";
}

void thrust_vectors_all_elements() {

	// initialize all ten integers of a device_vector to 1 
	thrust::device_vector<int> D(10, 1);
	// set the first seven elements of a vector to 9 
	thrust::fill(D.begin(), D.begin() + 7, 9);
	// initialize a host_vector with the first five elements of D 
	thrust::host_vector<int> H(D.begin(), D.begin() + 5);
	// set the elements of H to 0, 1, 2, 3, ... 
	thrust::sequence(H.begin(), H.end());
	// copy all of H back to the beginning of D 
	thrust::copy(H.begin(), H.end(), D.begin());
	//print D 
	for (int i = 0; i < D.size(); i++)
		std::cout << "D[" << i << "] = " << D[i] << std::endl;
}

void iterators_and_static_dispatching() {
	/* You can pass not only iterators like H.begin() by also raw pointers
	to thrust:: functions. A regular pointer is considered by the library to be
	pointing to CPU location (if it is in fact pointing to GPU this will cause problems).
	To tell the library that the pointer actually points to a GPU location,
	wrap it with thrust::device_ptr.
	*/

	size_t N = 10;
	// raw pointer to device memory 
	int * raw_ptr;
	cudaMalloc((void **)&raw_ptr, N * sizeof(int)); //allocates memory at GPU
	// wrap raw pointer with a device_ptr 
	thrust::device_ptr<int> dev_ptr(raw_ptr);
	// use device_ptr in thrust algorithms 
	thrust::fill(dev_ptr, dev_ptr + N, (int)0);

	// extracting the row pointer
	raw_ptr = thrust::raw_pointer_cast(dev_ptr);
}

// saxply is a BLAS function : y <- a * x + y
struct saxpy_functor {
	const float a;
	saxpy_functor(float _a) : a(_a) {}
	__host__ __device__
		float operator()(const float& x, const float& y) const {
		return a * x + y;
	}
};


void thrust_algorithms_transformations() {
	// allocate three device_vectors with 10 elements 
	thrust::device_vector<int> X(10);
	thrust::device_vector<int> Y(10);
	thrust::device_vector<int> Z(10);
	// initialize X to 0,1,2,3, .... 
	thrust::sequence(X.begin(), X.end());
	// compute Y = -X 
	thrust::transform(X.begin(), X.end(), Y.begin(), thrust::negate<int>());
	// fill Z with twos
	thrust::fill(Z.begin(), Z.end(), 2);
	// compute Y = X mod 2 
	thrust::transform(X.begin(), X.end(), Z.begin(), Y.begin(), thrust::modulus<int>());
	// replace all the ones in Y with tens 
	thrust::replace(Y.begin(), Y.end(), 1, 10);
	// print Y 
	thrust::copy(Y.begin(), Y.end(), std::ostream_iterator<int>(std::cout, "\n"));

	float A = 5;
	// Above the saw build-in transformations, now consider a user-defined transformation
	thrust::transform(X.begin(), X.end(), Y.begin(), Y.begin(), saxpy_functor(A));
	// thrust::transform only supports transformations with one or two input 
	// arguments(e.g.f(x) → y and f(x, x) → y).

}

void thrust_algorithms_reductions() {
	thrust::device_vector<int> D(10, 1);
	int sum = thrust::reduce(D.begin(), D.end(), (int)0 /*initial value*/, thrust::plus<int>());

	// count the 1s in a vector
	thrust::device_vector<int> vec(5, 0);
	vec[1] = 1; vec[3] = 1; vec[4] = 1;
	int result = thrust::count(vec.begin(), vec.end(), 1);

}

template <typename T> struct square {
	__host__ __device__
		T operator()(const T& x) const {
		return x * x;
	}
};

void thrust_algorithms_compute_norm_example() {
	// initialize host array 
	float x[4] = { 1.0, 2.0, 3.0, 4.0 };
	// transfer to device 
	thrust::device_vector<float> d_x(x, x + 4);
	float norm = std::sqrt(
		thrust::transform_reduce(d_x.begin(), d_x.end(), square<float>(), 0, thrust::plus<float>())
		);
	std::cout << norm << std::endl;
}

template<typename T>
struct absolute_value {
	__host__ __device__
		T operator()(const T &x) const {
		return x < T(0) ? -x : x;
	}
};

void sum_of_abs() {
	thrust::device_vector<int> X(10);
	thrust::sequence(X.begin(), X.end());
	thrust::transform(X.begin(), X.begin() + 5, X.begin(), thrust::negate<int>());
	int result = thrust::transform_reduce(X.begin(), X.end(), absolute_value<int>(), 0, thrust::plus<int>());
	thrust::copy(X.begin(), X.end(), std::ostream_iterator<int>(std::cout, " "));
	std::cout << "result = " << result << std::endl;
}


struct square_root : public thrust::unary_function < float, float > {
	__host__ __device__
		float operator()(float x) const {
		return sqrtf(x);
	}
};


struct even : public thrust::unary_function < int, int > {
	__host__ __device__
		int operator()(int x) {
		if (x % 2 == 0) return x;
		else return 1;
	}
};

struct root_of_sum_of_squares : public thrust::binary_function < float, float, float > {
	__host__ __device__
		float operator()(float x, float y) {
		return sqrt(x * x + y * x);
	}
};

struct erase_evens : public thrust::binary_function < float, float, int > {
	__host__ __device__
		float operator()(float x, int i) {
		if (i % 2 == 1)
			return x;
		else return 0;
	}
};



void norm_of_unrolled_vector() {


	int h_data[8] = { 1, 2, 1, 2, 1, 2, 1, 2 };
	thrust::device_vector<int> d_data(h_data, h_data + 8);
	thrust::device_vector<float> d_result(8);
	thrust::device_vector<float> final(8);

	thrust::adjacent_difference(d_data.begin(), d_data.end(), d_result.begin(), root_of_sum_of_squares());
	thrust::counting_iterator<int> count(8);
	thrust::transform(d_result.begin(), d_result.end(), count, d_result.begin(), erase_evens());
	//thrust::copy(final.begin(), final.end(), std::ostream_iterator<float>(std::cout, "\n"));
	std::cout << thrust::reduce(d_result.begin(), d_result.end()) << std::endl;

}



int main(void) {
	thrust::device_vector<float> device_pointer;
	device_pointer.resize(10);
	thrust::fill(device_pointer.begin(), device_pointer.end(), 0);
	float * raw_pointer = thrust::raw_pointer_cast(device_pointer.data());
	std::cout << raw_pointer[0] << std::endl;
	//Pull functor_pull(raw_pointer);
	//thrust::device_vector<int> offsets;
	//offsets.resize(camera_width * camera_heigth);
	//thrust::sequence(offsets.begin(), offsets.end());
	//thrust::for_each(offsets.begin(), offsets.end(), functor_pull);

	//float * output = thrust::raw_pointer_cast(&device_pointer[0]);

	

	/*
	Full documentation
	http://thrust.github.io/doc/group__transformations.html
	Tutorial
	http://docs.nvidia.com/cuda/thrust/#axzz3WiiNhwRB
	*/

	/*thrust_vectors_introduction();
	thrust_vectors_all_elements();
	iterators_and_static_dispatching();
	thrust_algorithms_transformations();
	thrust_algorithms_reductions();
	thrust_algorithms_compute_norm_example();*/

	//norm_of_unrolled_vector();
	float pull_moving_sum = 0;
	int moving_window_size = 5;
	std::queue<float> pull_error_history;
	for (size_t i = 0; i < 10; i++) {
		float pull_error = i;
		pull_error_history.push(pull_error);
		std::cout << "i = " << i << ", size = " << pull_error_history.size() << std::endl;
		pull_moving_sum = pull_moving_sum + pull_error;
		if (pull_error_history.size() > moving_window_size) {
			pull_moving_sum = pull_moving_sum - pull_error_history.front();
			pull_error_history.pop();
		}
		float pull_moving_average = pull_moving_sum / pull_error_history.size();
		std::cout << pull_moving_average << std::endl;
	}



	return 0;
}
