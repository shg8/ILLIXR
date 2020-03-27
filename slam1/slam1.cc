#include <chrono>
#include <thread>
#include <cmath>
#include <ctime>
#include "common/component.hh"
#include "common/switchboard.hh"
#include "common/data_format.hh"

using namespace ILLIXR;

class slam1 : public component {
public:
	/* Provide handles to slam1 */
	slam1(std::unique_ptr<reader_latest<camera_frame>>&& camera,
		  std::unique_ptr<writer<pose_type>>&& pose)
		: _m_camera{std::move(camera)}
		, _m_pose{std::move(pose)}
		, state{0}
	{
		start_time = std::chrono::system_clock::now();
	}

	virtual void _p_compute_one_iteration() override {
		using namespace std::chrono_literals;

		// Until we have pose prediction, we'll run our magical
		// make-believe SLAM system at an unreasonably high
		// frequency, like 120hz!
		std::this_thread::sleep_for(500ms);

		/* The component can read the latest value from its
		   subscription. */
		auto frame = _m_camera->get_latest_ro();
		if (frame) {
			state += frame->pixel[0];
		}

		/* Instead of allocating a new buffer with malloc/new, the
		   topic can optionally recycle old buffers (completing the
		   swap-chain). Unfortunately, this doesn't work yet.
		pose* buf = std::any_cast<pose*>(_m_pose->allocate());
		*/
		// std::cout << "Slam" << std::endl;

		// RT will delete this memory when it gets replaced with a newer value.
		pose_type* new_pose = new pose_type;
		std::chrono::duration<float> this_time = std::chrono::system_clock::now() - start_time;
		new_pose->time = std::chrono::system_clock::now();
		new_pose->position = Eigen::Vector3f{0, 0, 6};
		new_pose->orientation = generateDummyOrientation(this_time.count());

		/* Publish this buffer to the topic. */
		_m_pose->put(new_pose);
	}

	virtual ~slam1() override {
		/*
		  This developer is responsible for killing their processes
		  and deallocating their resources here.
		*/
	}

private:
	std::unique_ptr<reader_latest<camera_frame>> _m_camera;
	std::unique_ptr<writer<pose_type>> _m_pose;

	std::chrono::time_point<std::chrono::system_clock> start_time;
	
	int state;

	Eigen::Quaternionf generateDummyOrientation(float time) {
		float rollIsPitch = 0.3f * sinf( time * 1.5f ) * 0.0f;
		float yawIsRoll = 0;
		float pitchIsYaw = 0.3f * cosf( time * 1.5f ) * 0.0f;
		// printf("Yaw: %f\n", pitchIsYaw);

		//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
		double cy = cos(yawIsRoll * 0.5);
		double sy = sin(yawIsRoll * 0.5);
		double cp = cos(pitchIsYaw * 0.5);
		double sp = sin(pitchIsYaw * 0.5);
		double cr = cos(rollIsPitch * 0.5);
		double sr = sin(rollIsPitch * 0.5);
		
		float w = cy * cp * cr + sy * sp * sr;
		float x = cy * cp * sr - sy * sp * cr;
		float y = sy * cp * sr + cy * sp * cr;
		float z = sy * cp * cr - cy * sp * sr;

		return Eigen::Quaternionf{w, x, y, z};
	}
};

extern "C" component* create_component(switchboard* sb) {
	/* First, we declare intent to read/write topics. Switchboard
	   returns handles to those topics. */
	auto camera_ev = sb->subscribe_latest<camera_frame>("camera");
	auto pose_ev = sb->publish<pose_type>("slow_pose");

	/* This is the default pose, which will be published on the topic before SLAM does anythnig. */
	pose_type* new_pose = new pose_type{std::chrono::system_clock::now(), Eigen::Vector3f{0, 0, 6}, Eigen::Quaternionf{1, 0, 0, 0}};
	pose_ev->put(new_pose);

	auto this_slam1 = new slam1{std::move(camera_ev), std::move(pose_ev)};
	return this_slam1;
}
