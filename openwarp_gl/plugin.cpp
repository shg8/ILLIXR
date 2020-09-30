#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/extended_window.hpp"
#include "common/shader_util.hpp"
#include "utils/hmd.hpp"
#include "common/math_util.hpp"
#include "shaders/basic_shader.hpp"
#include "shaders/openwarp_shader.hpp"
#include "common/pose_prediction.hpp"

using namespace ILLIXR;

typedef void (*glXSwapIntervalEXTProc)(Display *dpy, GLXDrawable drawable, int interval);

const record_header openwarp_gpu_record {"openwarp_gpu", {
	{"iteration_no", typeid(std::size_t)},
	{"wall_time_start", typeid(std::chrono::high_resolution_clock::time_point)},
	{"wall_time_stop" , typeid(std::chrono::high_resolution_clock::time_point)},
	{"gpu_time_duration", typeid(std::chrono::nanoseconds)},
}};

class openwarp_gl_impl : public openwarp {

public:
	// Public constructor, create_component passes Switchboard handles ("plugs")
	// to this constructor. In turn, the constructor fills in the private
	// references to the switchboard plugs, so the component can read the
	// data whenever it needs to.
	openwarp_gl(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, xwin{pb->lookup_impl<xlib_gl_extended_window>()}
		, _m_hologram{sb->publish<hologram_input>("hologram_in")}
		, openwarp_gpu_logger{record_logger_}
		, mtp_logger{record_logger_}
	{ }

private:
	const std::shared_ptr<switchboard> sb;

	const std::shared_ptr<xlib_gl_extended_window> xwin;

	record_coalescer openwarp_gpu_logger;

	GLuint openwarpShaderProgram;

	// Eye sampler array
	GLuint eye_sampler;

	// Depth sampler
	GLuint depth_sampler;

	// Eye index uniform
	GLuint tw_eye_index_unif;

	// Near/far clip uniforms
	GLuint u_near_clip;
	GLuint u_far_clip;

	// VAOs
	GLuint tw_vao;

	// Position and UV attribute locations=
	GLuint flat_uv_attr;

	// Distortion mesh information
	GLuint num_distortion_vertices;
	GLuint num_distortion_indices;

	// Distortion mesh CPU buffers and GPU VBO handles
	HMD::mesh_coord3d_t* distortion_positions;
	GLuint distortion_positions_vbo;
	GLuint* distortion_indices;
	GLuint distortion_indices_vbo;
	HMD::uv_coord_t* distortion_uv0;
	GLuint distortion_uv0_vbo;

	HMD::uv_coord_t* flat_uv;
	GLuint flat_uv_vbo;

	GLuint u_render_inverse_p;
	GLuint u_render_inverse_v;

	GLuint u_warp_vp;

	// Basic perspective projection matrix
	Eigen::Matrix4f basicProjection;

	// Build a rectangular plane.
	void BuildMesh(size_t width, size_t height, std::vector<GLuint>& indices, std::vector<vertex_t>& vertices){
		
		// Compute the size of the vectors we'll need to store the
		// data, ahead of time.

		// width and height are not in # of verts, but in # of faces.
		size_t num_indices = 2 * 3 * width * height;
		size_t num_verts = (width + 1)*(height + 1);

		// Size the vectors accordingly
		indices.resize(num_indices);
		num_verts.resizse(num_verts);

		// Build indices.
		for ( int y = 0; y < height; y++ ) {
			for ( int x = 0; x < width; x++ ) {

				const int offset = ( y * width + x ) * 6;

				indices[offset + 0] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 1] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 2] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 1 ) );

				indices[offset + 3] = (GLuint)( ( y + 0 ) * ( width + 1 ) + ( x + 1 ) );
				indices[offset + 4] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 0 ) );
				indices[offset + 5] = (GLuint)( ( y + 1 ) * ( width + 1 ) + ( x + 1 ) );
			}
		}

		// Build vertices
		for (int y = 0; y < height + 1; y++){
			for (int x = 0; x < width + 1; x++){

				const int index = y * ( width + 1 ) + x;
				vertices[num_verts + index].x = ( -1.0f + ( (float)x / width ) );
				vertices[num_verts + index].y = ( -1.0f + 2.0f * ( ( height - (float)y ) / height );
				vertices[num_verts + index].z = 0.0f;
			}
		}
	}


public:

	virtual skip_option _p_should_skip() override {
		using namespace std::chrono_literals;
		// Sleep for approximately 90% of the time until the next vsync.
		// Scheduling granularity can't be assumed to be super accurate here,
		// so don't push your luck (i.e. don't wait too long....) Tradeoff with
		// MTP here. More you wait, closer to the display sync you sample the pose.

		// TODO: poll GLX window events
		std::this_thread::sleep_for(std::chrono::duration<double>(EstimateTimeToSleep(DELAY_FRACTION)));
		if(_m_eyebuffer->get_latest_ro()) {
			return skip_option::run;
		} else {
			// Null means system is nothing has been pushed yet
			// because not all components are initialized yet
			return skip_option::skip_and_yield;
		}
	}

	virtual void _p_one_iteration() override {
		warp(glfwGetTime());
	}

	virtual void _p_thread_setup() override {
		lastSwapTime = std::chrono::high_resolution_clock::now();

		// Generate reference HMD and physical body dimensions
    	HMD::GetDefaultHmdInfo(SCREEN_WIDTH, SCREEN_HEIGHT, &hmd_info);
		HMD::GetDefaultBodyInfo(&body_info);

		// Initialize the GLFW library, still need it to get time
		if(!glfwInit()){
			printf("Failed to initialize glfw\n");
		}

    	// Construct openwarp meshes and other data
    	Buildopenwarp(&hmd_info);

		// includes setting swap interval
		glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc);

		// set swap interval for 1
		glXSwapIntervalEXTProc glXSwapIntervalEXT = 0;
		glXSwapIntervalEXT = (glXSwapIntervalEXTProc) glXGetProcAddressARB((const GLubyte *)"glXSwapIntervalEXT");
		glXSwapIntervalEXT(xwin->dpy, xwin->win, 1);

		// Init and verify GLEW
		glewExperimental = GL_TRUE;
		if(glewInit() != GLEW_OK){
			printf("Failed to init GLEW\n");
			// clean up ?
			exit(0);
		}

		glEnable              ( GL_DEBUG_OUTPUT );
		glDebugMessageCallback( MessageCallback, 0 );

		// TODO: X window v-synch

		// Create and bind global VAO object
		glGenVertexArrays(1, &tw_vao);
    	glBindVertexArray(tw_vao);

    	openwarpShaderProgram = init_and_link(meshWarpVertexProgramGLSL, meshWarpFragmentProgramGLSL);
		// Acquire attribute and uniform locations from the compiled and linked shader program

    	distortion_pos_attr = glGetAttribLocation(openwarpShaderProgram, "vertexPosition");
		flat_uv_attr = glGetAttribLocation(openwarpShaderProgram, "flatUv");
    	distortion_uv0_attr = glGetAttribLocation(openwarpShaderProgram, "vertexUv0");
    	distortion_uv1_attr = glGetAttribLocation(openwarpShaderProgram, "vertexUv1");
    	distortion_uv2_attr = glGetAttribLocation(openwarpShaderProgram, "vertexUv2");
    	tw_start_transform_unif = glGetUniformLocation(openwarpShaderProgram, "openwarpStartTransform");
    	tw_end_transform_unif = glGetUniformLocation(openwarpShaderProgram, "openwarpEndTransform");
    	tw_eye_index_unif = glGetUniformLocation(openwarpShaderProgram, "ArrayLayer");
    	eye_sampler = glGetUniformLocation(openwarpShaderProgram, "Texture");

		u_near_clip = glGetUniformLocation(openwarpShaderProgram, "u_NearClip");
		u_far_clip = glGetUniformLocation(openwarpShaderProgram, "u_FarClip");

		u_render_inverse_p = glGetUniformLocation(openwarpShaderProgram, "u_renderInverseP");
		u_render_inverse_v = glGetUniformLocation(openwarpShaderProgram, "u_renderInverseV");

		u_warp_vp = glGetUniformLocation(openwarpShaderProgram, "u_warpVP");

		depth_sampler = glGetUniformLocation(openwarpShaderProgram, "_Depth");

		if(glGetError())
		{
			printf("uniforms/attribs error\n");
			// std::cout << "attr: " << distortion_uv2_attr << std::endl;
			abort();
		}
		
		if(distortion_pos_attr != -1){
			// Config distortion mesh position vbo
			glGenBuffers(1, &distortion_positions_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, distortion_positions_vbo);
			glBufferData(GL_ARRAY_BUFFER, HMD::NUM_EYES * (num_distortion_vertices * 3) * sizeof(GLfloat), distortion_positions, GL_STATIC_DRAW);
			glVertexAttribPointer(distortion_pos_attr, 3, GL_FLOAT, GL_FALSE, 0, 0);
		}

		if(glGetError())
		{
			printf("distort pos error\n");
			// std::cout << "attr: " << distortion_uv2_attr << std::endl;
			abort();
		}
		
		if(flat_uv_vbo != -1) {
			// Config flat_uv_vbo
			glGenBuffers(1, &flat_uv_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, flat_uv_vbo);
			glBufferData(GL_ARRAY_BUFFER, HMD::NUM_EYES * (num_distortion_vertices * 2) * sizeof(GLfloat), flat_uv, GL_STATIC_DRAW);
			glVertexAttribPointer(flat_uv_vbo, 2, GL_FLOAT, GL_FALSE, 0, 0);
		}

		if(distortion_uv0_attr != -1) {
			// Config distortion uv0 vbo
			glGenBuffers(1, &distortion_uv0_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, distortion_uv0_vbo);
			glBufferData(GL_ARRAY_BUFFER, HMD::NUM_EYES * (num_distortion_vertices * 2) * sizeof(GLfloat), distortion_uv0, GL_STATIC_DRAW);
			glVertexAttribPointer(distortion_uv0_attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
		}

		if(distortion_uv1_attr != -1) {
			// Config distortion uv1 vbo
			glGenBuffers(1, &distortion_uv1_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, distortion_uv1_vbo);
			glBufferData(GL_ARRAY_BUFFER, HMD::NUM_EYES * (num_distortion_vertices * 2) * sizeof(GLfloat), distortion_uv1, GL_STATIC_DRAW);
			glVertexAttribPointer(distortion_uv1_attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
		}

		if(distortion_uv2_attr != -1) {
			// Config distortion uv2 vbo
			glGenBuffers(1, &distortion_uv2_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, distortion_uv2_vbo);
			glBufferData(GL_ARRAY_BUFFER, HMD::NUM_EYES * (num_distortion_vertices * 2) * sizeof(GLfloat), distortion_uv2, GL_STATIC_DRAW);
			glVertexAttribPointer(distortion_uv2_attr, 2, GL_FLOAT, GL_FALSE, 0, 0);
		}

		if(glGetError())
		{
			printf("VBO error\n");
			// std::cout << "attr: " << distortion_uv2_attr << std::endl;
			abort();
		}

		// Config distortion mesh indices vbo
		glGenBuffers(1, &distortion_indices_vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, distortion_indices_vbo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_distortion_indices * sizeof(GLuint), distortion_indices, GL_STATIC_DRAW);

		

		glXMakeCurrent(xwin->dpy, None, NULL);
	}

	Eigen::Matrix4f getEyeballMatrix(int eye, const Eigen::Vector3f& head_pos, const Eigen::Quaternionf& head_rotation, float ipd) {
		// Offset of eyeball from pose
		auto eyeball = Eigen::Vector3f((eye == 0 ? -ipd/2.0f : ipd/2.0f), 0, 0);

		auto head_rotation_matrix = head_rotation.toRotationMatrix();

		// Apply head rotation to eyeball offset vector
		eyeball = head_rotation_matrix * eyeball;

		// Apply head position to eyeball
		eyeball += head_pos;

		// Build our eye matrix from the pose's position + orientation.
		Eigen::Matrix4f eye_matrix = Eigen::Matrix4f::Identity();
		eye_matrix.block<3,1>(0,3) = eyeball; // Set position to eyeball's position
		eye_matrix.block<3,3>(0,0) = head_rotation_matrix;

		return eye_matrix;
	}

	virtual void warp([[maybe_unused]] float time) {
		glXMakeCurrent(xwin->dpy, xwin->win, xwin->glc);

		glBindFramebuffer(GL_FRAMEBUFFER,0);
		glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
		glClearColor(0, 0, 0, 0);
    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
		glDepthFunc(GL_LEQUAL);

		auto most_recent_frame = _m_eyebuffer->get_latest_ro();
		// This should be null-checked in _p_should_skip
		assert(most_recent_frame);

		// Use the openwarp program
		glUseProgram(openwarpShaderProgram);

		//double cursor_x, cursor_y;
		//glfwGetCursorPos(window, &cursor_x, &cursor_y);

		// Generate "starting" view matrix, from the pose
		// sampled at the time of rendering the frame.
		Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();
		viewMatrix.block(0,0,3,3) = most_recent_frame->render_pose.pose.orientation.toRotationMatrix();
		// math_util::view_from_quaternion(&viewMatrix, most_recent_frame->render_pose.pose.orientation);

		// We simulate two asynchronous view matrices,
		// one at the beginning of display refresh,
		// and one at the end of display refresh.
		// The distortion shader will lerp between
		// these two predictive view transformations
		// as it renders across the horizontal view,
		// compensating for display panel refresh delay (wow!)
		Eigen::Matrix4f viewMatrixBegin = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f viewMatrixEnd = Eigen::Matrix4f::Identity();

		// TODO: Right now, this samples the latest pose published to the "pose" topic.
		// However, this should really be polling the high-frequency pose prediction topic,
		// given a specified timestamp!
		const fast_pose_type latest_pose = pp->get_fast_pose();
		viewMatrixBegin.block(0,0,3,3) = latest_pose.pose.orientation.toRotationMatrix();

		// TODO: We set the "end" pose to the same as the beginning pose, because panel refresh is so tiny
		// and we don't need to visualize this right now (we also don't have prediction setup yet!)
		viewMatrixEnd = viewMatrixBegin;

		// Calculate the openwarp transformation matrices.
		// These are a product of the last-known-good view matrix
		// and the predictive transforms.
		Eigen::Matrix4f openwarpStartTransform4x4;
		Eigen::Matrix4f openwarpEndTransform4x4;

		// Calculate openwarp transforms using predictive view transforms
		CalculateopenwarpTransform(openwarpStartTransform4x4, basicProjection, viewMatrix, viewMatrixBegin);
		CalculateopenwarpTransform(openwarpEndTransform4x4, basicProjection, viewMatrix, viewMatrixEnd);

		if(tw_start_transform_unif != -1)
			glUniformMatrix4fv(tw_start_transform_unif, 1, GL_FALSE, (GLfloat*)(openwarpStartTransform4x4.data()));
		if(tw_end_transform_unif != -1)
			glUniformMatrix4fv(tw_end_transform_unif, 1, GL_FALSE,  (GLfloat*)(openwarpEndTransform4x4.data()));

		glUniform1i(eye_sampler, 0);
		glUniform1i(depth_sampler, 1);

		glUniform1f(u_near_clip, 0.03f);
		glUniform1f(u_far_clip, 20.0f);

		glUniformMatrix4fv(u_render_inverse_p, 1, GL_FALSE, (GLfloat*)(most_recent_frame->render_projection.inverse().eval().data()));

		Eigen::Matrix4f renderedInverseViewMatrices[2];
		Eigen::Matrix4f warpInverseViewMatrices[2];

		for(int eye_idx = 0; eye_idx < HMD::NUM_EYES; eye_idx++){
			// renderedInverseViewMatrices[eye_idx] = getEyeballMatrix(eye_idx, most_recent_frame->render_pose.pose.position,
			// 												most_recent_frame->render_pose.pose.orientation,
			// 												body_info.interpupillaryDistance);

			renderedInverseViewMatrices[eye_idx] = most_recent_frame->view_matrices[eye_idx].inverse();
			
			warpInverseViewMatrices[eye_idx] = getEyeballMatrix(eye_idx, latest_pose.pose.position,
															latest_pose.pose.orientation,
															body_info.interpupillaryDistance);
		}
		


		glBindVertexArray(tw_vao);

		auto gpu_start_wall_time = std::chrono::high_resolution_clock::now();

		GLuint query;
		GLuint64 elapsed_time = 0;
		glGenQueries(1, &query);
		glBeginQuery(GL_TIME_ELAPSED, query);

		if(glGetError())
		{
			printf("Error before eye loop\n");
			// std::cout << "attr: " << distortion_uv2_attr << std::endl;
			abort();
		}

		// Loop over each eye.
		for(int eye = 0; eye < HMD::NUM_EYES; eye++){
			
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, most_recent_frame->texture_handles[eye]);

			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, most_recent_frame->depth_handles[eye]);

			if(glGetError())
			{
				printf("Error after binding textures\n");
				// std::cout << "attr: " << distortion_uv2_attr << std::endl;
				abort();
			}

			glUniformMatrix4fv(u_render_inverse_v, 1, GL_FALSE, (GLfloat*)(renderedInverseViewMatrices[eye].data()));
			
			auto vp = most_recent_frame->render_projection * warpInverseViewMatrices[0].inverse();
			glUniformMatrix4fv(u_warp_vp, 1, GL_FALSE, (GLfloat*)(vp.eval().data()));

			// The distortion_positions_vbo GPU buffer already contains
			// the distortion mesh for both eyes! They are contiguously
			// laid out in GPU memory. Therefore, on each eye render,
			// we set the attribute pointer to be offset by the full
			// eye's distortion mesh size, rendering the correct eye mesh
			// to that region of the screen. This prevents re-uploading
			// GPU data for each eye.
			if(distortion_pos_attr != -1) {
				glBindBuffer(GL_ARRAY_BUFFER, distortion_positions_vbo);
				glVertexAttribPointer(distortion_pos_attr, 3, GL_FLOAT, GL_FALSE, 0, (void*)(eye * num_distortion_vertices * sizeof(HMD::mesh_coord3d_t)));
				glEnableVertexAttribArray(distortion_pos_attr);
			}

			// We do the exact same thing for the UV GPU memory.
			if(distortion_uv0_attr != -1) {
				glBindBuffer(GL_ARRAY_BUFFER, distortion_uv0_vbo);
				glVertexAttribPointer(distortion_uv0_attr, 2, GL_FLOAT, GL_FALSE, 0, (void*)(eye * num_distortion_vertices * sizeof(HMD::mesh_coord2d_t)));
				glEnableVertexAttribArray(distortion_uv0_attr);
			}

			// We do the exact same thing for the UV GPU memory.
			if(distortion_uv1_attr != -1) {
				glBindBuffer(GL_ARRAY_BUFFER, distortion_uv1_vbo);
				glVertexAttribPointer(distortion_uv1_attr, 2, GL_FLOAT, GL_FALSE, 0, (void*)(eye * num_distortion_vertices * sizeof(HMD::mesh_coord2d_t)));
				glEnableVertexAttribArray(distortion_uv1_attr);
			}

			// We do the exact same thing for the UV GPU memory.
			if(distortion_uv2_attr != -1) {
				glBindBuffer(GL_ARRAY_BUFFER, distortion_uv2_vbo);
				glVertexAttribPointer(distortion_uv2_attr, 2, GL_FLOAT, GL_FALSE, 0, (void*)(eye * num_distortion_vertices * sizeof(HMD::mesh_coord2d_t)));
				glEnableVertexAttribArray(distortion_uv2_attr);
			}

			// We do the exact same thing for the UV GPU memory.
			glBindBuffer(GL_ARRAY_BUFFER, flat_uv_vbo);
			glVertexAttribPointer(flat_uv_attr, 2, GL_FLOAT, GL_FALSE, 0, (void*)(eye * num_distortion_vertices * sizeof(HMD::mesh_coord2d_t)));
			glEnableVertexAttribArray(flat_uv_attr);

			if(glGetError())
			{
				printf("Error after setting buffers\n");
				// std::cout << "attr: " << distortion_uv2_attr << std::endl;
				abort();
			}


			// Interestingly, the element index buffer is identical for both eyes, and is
			// reused for both eyes. Therefore glDrawElements can be immediately called,
			// with the UV and position buffers correctly offset.
			glDrawElements(GL_TRIANGLES, num_distortion_indices, GL_UNSIGNED_INT, (void*)0);
		}

		glEndQuery(GL_TIME_ELAPSED);

#ifndef NDEBUG
		auto delta = std::chrono::high_resolution_clock::now() - most_recent_frame->render_time;
		printf("\033[1;36m[openwarp]\033[0m Time since render: %3fms\n", (float)(delta.count() / 1000000.0));
		if(delta > vsync_period)
		{
			printf("\033[0;31m[openwarp: CRITICAL]\033[0m Stale frame!\n");
		}
		printf("\033[1;36m[openwarp]\033[0m Warping from swap %d\n", most_recent_frame->swap_indices[0]);
#endif
		// Call Hologram
		auto hologram_params = new hologram_input;
		hologram_params->seq = ++_hologram_seq;
		_m_hologram->put(hologram_params);

		// Call swap buffers; when vsync is enabled, this will return to the CPU thread once the buffers have been successfully swapped.
		// TODO: GLX V SYNCH SWAP BUFFER
		[[maybe_unused]] auto beforeSwap = glfwGetTime();

		glXSwapBuffers(xwin->dpy, xwin->win);

		// The swap time needs to be obtained and published as soon as possible
		lastSwapTime = std::chrono::high_resolution_clock::now();

		// Now that we have the most recent swap time, we can publish the new estimate.
		_m_vsync_estimate->put(new time_type(GetNextSwapTimeEstimate()));

#ifndef NDEBUG
		auto afterSwap = glfwGetTime();
		printf("\033[1;36m[openwarp]\033[0m Swap time: %5fms\n", (float)(afterSwap - beforeSwap) * 1000);
#endif

		// retrieving the recorded elapsed time
		// wait until the query result is available
		int done = 0;
		glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);
		while (!done) {
			std::this_thread::yield();
			glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);
		}

		// get the query result
		glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
		openwarp_gpu_logger.log(record{openwarp_gpu_record, {
			{iteration_no},
			{gpu_start_wall_time},
			{std::chrono::high_resolution_clock::now()},
			{std::chrono::nanoseconds(elapsed_time)},
		}});

		mtp_logger.log(record{mtp_record, {
			{iteration_no},
			{std::chrono::high_resolution_clock::now()},
			{latest_pose.pose.sensor_time},
		}});

#ifndef NDEBUG
		// TODO (implement-logging): When we have logging infra, delete this code.
		// Compute time difference between current post-vsync time and the time of the most recent
		// imu sample that was used to generate the predicted pose. This is the MTP, assuming good prediction
		// was used to compute the most recent fast pose.
		std::chrono::duration<double,std::nano> mtp = (lastSwapTime - latest_pose.pose.sensor_time);
		std::chrono::duration<double,std::nano> predict_to_display = (lastSwapTime - latest_pose.predict_computed_time);

		printf("\033[1;36m[openwarp]\033[0m Motion-to-display latency: %3f ms\n", (float)(mtp.count() / 1000000.0));
		printf("\033[1;36m[openwarp]\033[0m Prediction-to-display latency: %3f ms\n", (float)(predict_to_display.count() / 1000000.0));

		std::cout<< "openwarp estimating: " << std::chrono::duration_cast<std::chrono::milliseconds>(GetNextSwapTimeEstimate() - lastSwapTime).count() << "ms in the future" << std::endl;
#endif
	}

	virtual ~openwarp_gl() override {
		// TODO: Need to cleanup resources here!
		glXMakeCurrent(xwin->dpy, None, NULL);
 		glXDestroyContext(xwin->dpy, xwin->glc);
 		XDestroyWindow(xwin->dpy, xwin->win);
 		XCloseDisplay(xwin->dpy);
	}
};

PLUGIN_MAIN(openwarp_gl)
