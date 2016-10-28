// Adapted from Section 8.2 of "Cuda by Example" 2010 
// Compile it as standalone with: 
// /Developer/NVIDIA/CUDA-6.0/bin/nvcc -arch=sm_30 -o ogltest ogltest.cu -lglut
// 

#if __unix__
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 

#if __unix__
#include <GL/gl.h> 
#include <GL/glut.h>
#else
#include <OpenGL/gl.h> 
#include <OpenGL/glut.h>
#endif

#include <cuda_gl_interop.h> 
#include <thrust/device_ptr.h>
#include <thrust/sort.h>

//function to check if a CUDA error has been raised
#define cutool_checkErrorMsg()   __cutool_checkErrorMsg( __FILE__, __LINE__)
inline void __cutool_checkErrorMsg( const char *file, const int line )
{
	cudaError cudares = cudaGetLastError();
    if( cudaSuccess != cudares)
	{
		fprintf(stderr, "CUDA Runtime API error (file %s, line %i): %s.\n",file, line, cudaGetErrorString( cudares) );
        exit(-1);
    }
}

#define GET_PROC_ADDRESS( str ) glXGetProcAddress( (const GLubyte *)str ) 

static void HandleError( cudaError_t err, const char *file,  int line ) { 
    if (err != cudaSuccess) { 
            printf( "%s in %s at line %d\n", cudaGetErrorString( err ),  file, line ); 
            exit( EXIT_FAILURE ); 
    } 
} 
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ )) 

#define     DIM    512 

GLuint  bufferObj; 
cudaGraphicsResource *resource; 

struct sort_functor
{
  __host__ __device__
    bool operator()(uchar4 left, uchar4 right) const
    {
      return (left.y < right.y);
    }
};


// create a green/black pattern
__global__ void kernel( uchar4 *ptr ) { 
// map from threadIdx/BlockIdx to pixel position 
  int x = threadIdx.x + blockIdx.x * blockDim.x; 
  int y = threadIdx.y + blockIdx.y * blockDim.y; 
  int offset = x + y * blockDim.x * gridDim.x; 

// now calculate the value at that position 
  float fx = x/(float)DIM - 0.5f; 
  float fy = y/(float)DIM - 0.5f; 
  unsigned char   green = 128 + 127 * sin( abs(fx*100) - abs(fy*100) ); 

// accessing uchar4 vs unsigned char* 
  ptr[offset].x = 0; 
  ptr[offset].y = green; 
  ptr[offset].z = 0; 
  ptr[offset].w = 255; 
} 


#ifdef CPU_DATA
void fill_data( uchar4 *ptr ) {
    for(int x=0; x<DIM; x++){
        for(int y=0; y<DIM; y++){
            int offset = x + y * DIM;
            float fx = x/(float)DIM - 0.5f; 
            float fy = y/(float)DIM - 0.5f; 
            unsigned char   green = 128 + 127 * sin( abs(fx*100) - abs(fy*100) ); 
          
          // accessing uchar4 vs unsigned char* 
            ptr[offset].x = 0; 
            ptr[offset].y = green; 
            ptr[offset].z = 0; 
            ptr[offset].w = 255;           
        }
    } 
} 
#endif


static void draw_func( void ){
#ifndef CHECK_OPENGL_WORKS 
  glDrawPixels( DIM, DIM, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
#else
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0,0,0,1);
    float x=-.5, y=-.5;
    float w=1, h=1;    
    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
        glVertex3f(x,y,0);
        glVertex3f(x+w,y,0);
        glVertex3f(x+w,y+h,0);
        glVertex3f(x,y+h,0);
    glEnd();
#endif
    
  glutSwapBuffers(); 
}

static void sort_pixels(){
    cudaGraphicsMapResources( 1, &resource, NULL ); 
        uchar4* devPtr; 
        size_t  size; 
        cudaGraphicsResourceGetMappedPointer( (void**)&devPtr, &size, resource); 
        thrust::device_ptr<uchar4> tptr = thrust::device_pointer_cast(devPtr);
        thrust::sort(tptr, tptr+(DIM*DIM), sort_functor());
    cudaGraphicsUnmapResources( 1, &resource, NULL ); 
    draw_func();
}

static void key_func( unsigned char key, int x, int y ) { 
  switch (key) { 
    case 27: 
        HANDLE_ERROR( cudaGraphicsUnregisterResource( resource ) ); 
        glBindBuffer( GL_PIXEL_UNPACK_BUFFER_ARB, 0 ); 
        glDeleteBuffers( 1, &bufferObj ); 
        exit(0); 
        break;
    case 32: /// spacebar
        sort_pixels();
        break;
    default:
        break;
  } 
} 



int main(int argc, char *argv[]) { 
    cudaGLSetGLDevice( 0 ); 
    glutInit( &argc, argv ); 
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA ); 
    glutInitWindowSize( DIM, DIM ); 
    glutCreateWindow( "cuda_thrust_sort" );

#ifdef CPU_DATA
    /// Do the same operation on CPU instead of GPU
    uchar4 data[DIM*DIM];
    fill_data(data);
#else
    uchar4* data = NULL;
#endif
  
    ///--- Generate an OpenGL pixel buffer to write stuff on
    glGenBuffers( 1, &bufferObj ); 
    glBindBuffer( GL_PIXEL_UNPACK_BUFFER_ARB, bufferObj ); 
    glBufferData( GL_PIXEL_UNPACK_BUFFER_ARB, DIM * DIM * 4, data, GL_DYNAMIC_DRAW_ARB ); 
    
    ///--- Registers it with cuda (devPtr) so that we can process it  
    cudaGraphicsGLRegisterBuffer( &resource, bufferObj, cudaGraphicsMapFlagsNone ); 
    cudaGraphicsMapResources( 1, &resource, NULL ); 
    uchar4* devPtr; ///< cuda accessed pointer
    size_t  size; 
    cudaGraphicsResourceGetMappedPointer( (void**)&devPtr, &size, resource); 
    
    ///--- Do something with it 
    dim3    grid(DIM/16,DIM/16); 
    dim3    threads(16,16); 
    kernel<<<grid,threads>>>( devPtr ); 
    cutool_checkErrorMsg();
    cudaGraphicsUnmapResources( 1, &resource, NULL ); 
    
    ///--- Display result using OpenGL directly
    glutKeyboardFunc( key_func ); 
    glutDisplayFunc( draw_func ); 
    glutMainLoop(); 
} 
