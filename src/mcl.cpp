#include "mcl.h"
#include <cmath>
#include <iostream>

const int PARTICLE_QUANTITY = 500; // tune

pros::Distance dNorth(3);
pros::Distance dEast(10);
pros::Distance dSouth(13); // 13.75
pros::Distance dWest(16);



namespace MCL
{
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<float> noise_x(0.0f,
                                            0.1f); // Mean 0, std dev 0.1 inches
    std::normal_distribution<float> noise_y(0.0f,
                                            0.1f);           // Mean 0, std dev 0.1 inches
    std::normal_distribution<float> noise_theta(0.0f, 5.0f); // Std dev 5.0 degrees

    lemlib::Pose lastOdomPose(0, 0, 0);

    pros::Task *mclTaskHandle = nullptr;
    bool mclRunning = false;

    const int MCL_DELAY = 20;              // Run at 50Hz
    const float FIELD_DIMENSIONS = 144.0f; // Corrected to 144 inches

    const float NORTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float NORTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float SOUTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float SOUTH_SENSOR_Y_OFFSET = 8.0f; // Example offset, adjust as needed

    const float EAST_SENSOR_X_OFFSET = 6.5f; // Example offset, adjust as needed
    const float EAST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float WEST_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float WEST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    bool useNorthSensor = false;
    bool useSouthSensor = true;
    bool useEastSensor = true;
    bool useWestSensor = false;

    const float sigma_close_range = 0.3f;          // Sigma for distances below 200mm (approx 7.87 inches)
    const float sigma_far_range = 1.0f;            // Sigma for distances above 200mm
    const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

    // Add variables for update frequency control
    int lastUpdateTime = 0;                   // Timestamp of the last MCL update
    const int UPDATE_INTERVAL = 100;          // Update interval in milliseconds (1 second)
    lemlib::Pose lastUpdatedPose(0, 0, 0);    // Last pose at which MCL was updated
    const float MIN_MOTION_THRESHOLD = 0.25f; // Minimum motion in inches to trigger update

    const float MOTION_NOISE_THRESHOLD = 0.25f; // Threshold to consider motion as static
    const float MIN_MOTION_FOR_NOISE = 0.5f;    // Minimum motion to add noise

    const float UNIFORM_WEIGHT_FACTOR = 0.0001f; // Small uniform weight factor, tune as needed
    const float MIN_WEIGHT = 0.1f;               // Minimum weight floor for particles
    const int RESAMPLING_INTERVAL = 10;          // Only resample every 10th update

    // Add variables to store previous sensor readings
    float prev_north_dist = -1.0f; // Initialize with an out-of-range value
    float prev_south_dist = -1.0f;
    float prev_east_dist = -1.0f;
    float prev_west_dist = -1.0f;
    const float DISTANCE_CHANGE_THRESHOLD = 0.25f; // Adjust based on sensor noise

    lemlib::Pose filteredPose(0, 0, 0);

    const float FILTER_ALPHA = 0.3f;          // Smoothing factor (0.3 = 30% new, 70% old)
    const float ODOMETRY_TRUST_FACTOR = 0.5f; // 0.5 = equal trust in odometry and sensors




    void getSensorValues() {

        // 1. Get raw readings (converted to inches)
        double rawEast = dEast.get() / 25.4;
        double rawSouth = dSouth.get() / 25.4;

        // 2. Apply physical offsets 
        // If the sensor is 5 inches from the center, add that to the reading
        // to get the total distance from the robot's center to the wall.
        double totalEastDist = rawEast + EAST_SENSOR_X_OFFSET;
        double totalSouthDist = rawSouth + SOUTH_SENSOR_Y_OFFSET;

        // 3. Simple Output
        std::cout << "--- Sensor Data ---" << std::endl;
        std::cout << "Total East (with offset): " << totalEastDist << " in" << std::endl;
        
        std::cout << "Total South (with offset): " << totalSouthDist << " in" << std::endl;


    }

    void initializeParticles(const lemlib::Pose &initialPose)
    {
        particles.resize(PARTICLE_QUANTITY);
        lastOdomPose = initialPose;

        std::normal_distribution<float> x_dist(initialPose.x, 1.0);
        std::normal_distribution<float> y_dist(initialPose.y, 1.0);
        std::normal_distribution<float> theta_dist(initialPose.theta, 0.0);

        for (auto &particle : particles)
        {
            particle = Particle(lemlib::Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
                                1.0f / PARTICLE_QUANTITY);
        }
    }

    void motionUpdate(const lemlib::Pose &globalOdomDelta) // Rename for clarity, maybe do stuff with local/global coordinates here?
    {
        float motion_magnitude = std::sqrt(std::pow(globalOdomDelta.x, 2) + std::pow(globalOdomDelta.y, 2));
        bool add_noise = (motion_magnitude > MIN_MOTION_FOR_NOISE);

        std::normal_distribution<float> motion_noise(0.0, 0.05);
        std::normal_distribution<float> rotation_noise(0.0, 0.05);

        for (auto &particle : particles)
        {
            float noise_scale = add_noise ? std::min(1.0f, motion_magnitude / 2.0f) : 0.0f;

            // DIRECT ADDITION (Since delta is already global)
            particle.pose.x += globalOdomDelta.x + (noise_scale * motion_noise(gen));
            particle.pose.y += globalOdomDelta.y + (noise_scale * motion_noise(gen));

            // Theta logic remains the same
            particle.pose.theta += globalOdomDelta.theta + (noise_scale * rotation_noise(gen));

            // Normalization...
            particle.pose.theta = fmod(particle.pose.theta, 360.0f);
            if (particle.pose.theta < 0)
                particle.pose.theta += 360.0f;
        };
    }

    float predictSensorReading(const lemlib::Pose &particlePose, const char direction)
    {
        float half_dimension = FIELD_DIMENSIONS / 2.0f;
        float local_x = 0.0f; // Right (+), Left (-)
        float local_y = 0.0f; // Forward (+), Backward (-)

        float theta = particlePose.theta * M_PI / 180.0f; // Robot orientation in radians

        // Define the sensor's FOV (±12° from center)
        const float FOV_HALF_ANGLE = 12.0f * M_PI / 180.0f; // 12° in radians, unused rn

        // Direction-specific center angle (assuming 0° is north)
        float sensor_angle = 0.0f;
        switch (direction)
        {
        case 'N':                            // Front Sensor
            local_x = NORTH_SENSOR_X_OFFSET; // e.g., 0.0
            local_y = NORTH_SENSOR_Y_OFFSET; // e.g., 5.0
            sensor_angle = 0.0f;             // Points Forward (0 deg relative to robot)
            break;
        case 'W':                            // Left Sensor
            local_x = -WEST_SENSOR_X_OFFSET; // e.g., -5.0 (Negative for Left)
            local_y = WEST_SENSOR_Y_OFFSET;  // e.g., 0.0
            sensor_angle = -M_PI / 2.0f;     // Points Left (-90 deg relative to robot)
            break;
        case 'E':                           // Right Sensor
            local_x = EAST_SENSOR_X_OFFSET; // e.g., 5.0
            local_y = EAST_SENSOR_Y_OFFSET;
            sensor_angle = M_PI / 2.0f;
            break;
        case 'S': // Back Sensor
            local_x = SOUTH_SENSOR_X_OFFSET;
            local_y = -SOUTH_SENSOR_Y_OFFSET; // Negative for Back
            sensor_angle = M_PI;
            break;
        default:
            return -1.0f;
        }

        // Handle sensor by rotating local coordinates CW by theta
        float sensor_x = particlePose.x + (local_x * cos(theta) + local_y * sin(theta));
        float sensor_y = particlePose.y + (local_y * cos(theta) - local_x * sin(theta));

        float center_angle = sensor_angle + theta;

        float distances[4];

        // Simplified raycasting

        // Parametric line equations: x = sensor_x + d * sin(angle), y = sensor_y + d * cos(angle)
        // Find intersection with boundaries (d = distance to boundary)

        // Calculate distances to the north boundary (y=72)
        float d_north = (half_dimension - sensor_y) / cos(center_angle);
        distances[0] = (d_north > 0 && std::isfinite(d_north)) ? d_north : std::numeric_limits<float>::max();

        // Calculate distances to the south boundary (y=-72)
        float d_south = (-half_dimension - sensor_y) / cos(center_angle);
        distances[1] = (d_south > 0 && std::isfinite(d_south)) ? d_south : std::numeric_limits<float>::max();

        float d_east = (half_dimension - sensor_x) / sin(center_angle);
        distances[2] = (d_east > 0 && std::isfinite(d_east)) ? d_east : std::numeric_limits<float>::max();

        // West boundary (x = -72)
        float d_west = (-half_dimension - sensor_x) / sin(center_angle);
        distances[3] = (d_west > 0 && std::isfinite(d_west)) ? d_west : std::numeric_limits<float>::max();

        // Find the minimum positive distance within FOV
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < 4; ++i)
        {
            if (distances[i] > 0 && distances[i] < min_distance)
            {
                min_distance = distances[i];
            }
        }

        // If no valid intersection, return a large value
        if (min_distance == std::numeric_limits<float>::max())
        {
            return 200.0f; // Default to max field distance
        }

        return min_distance;
    }

    void measurementUpdate(float north_dist, float south_dist, float east_dist, float west_dist) //! Add cases for more than west sensor, use north sensor
    {

        float total_weight = 0.0f;
        for (auto &particle : particles)
        {
            float particle_weight = 1.0f;
            int valid_readings = 0;

            auto getSigma = [&](float predicted_distance)
            {
                return 2.0f; // Wide sigma to account for FOV uncertainty
            };

            if (west_dist >= 0)
            {
                float predicted_west_dist = predictSensorReading(particle.pose, 'W');
                float west_diff = std::abs(predicted_west_dist - west_dist);
                float sigma = getSigma(predicted_west_dist);
                float west_likelihood = std::exp(-(west_diff * west_diff) / (2.0f * sigma * sigma));
                particle_weight *= west_likelihood;
                valid_readings++;
            }

            if (north_dist >= 0)
            {
                float predicted_north_dist = predictSensorReading(particle.pose, 'N');
                float north_diff = std::abs(predicted_north_dist - north_dist);
                float sigma = getSigma(predicted_north_dist);
                float north_likelihood = std::exp(-(north_diff * north_diff) / (2.0f * sigma * sigma));
                particle_weight *= north_likelihood;
                valid_readings++;
            }

            if (south_dist >= 0)
            {
                float predicted_south_dist = predictSensorReading(particle.pose, 'S');
                float south_diff = std::abs(predicted_south_dist - south_dist);
                float sigma = getSigma(predicted_south_dist);
                float south_likelihood = std::exp(-(south_diff * south_diff) / (2.0f * sigma * sigma));
                particle_weight *= south_likelihood;
                valid_readings++;
            }

            if (valid_readings > 0)
            {
                particle.weight = std::max(particle_weight, 0.001f);
            }
            else
            {
                particle.weight = 0.001f;
            }
            total_weight += particle.weight;
        }

        if (total_weight > 0)
        {
            for (auto &particle : particles)
            {
                particle.weight /= total_weight;
            }
        }
    
        prev_north_dist = north_dist;
        prev_west_dist = west_dist;
        prev_south_dist = south_dist;
        prev_east_dist = east_dist;
    }

    std::vector<Particle> weightedResample(const std::vector<Particle> &particles)
    {
        std::vector<Particle> new_particles(PARTICLE_QUANTITY);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);

        // Compute cumulative weights
        std::vector<float> cumulative_weights(PARTICLE_QUANTITY);
        cumulative_weights[0] = particles[0].weight;
        for (size_t i = 1; i < PARTICLE_QUANTITY; ++i)
        {
            cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].weight;
        }

        // Systematic resampling
        float step = 1.0f / PARTICLE_QUANTITY;
        float r = dist(gen) * step; // Initial random offset
        size_t index = 0;

        for (size_t m = 0; m < PARTICLE_QUANTITY; ++m)
        {
            float U = r + m * step;
            while (index < PARTICLE_QUANTITY - 1 && U > cumulative_weights[index])
            {
                ++index;
            }
            new_particles[m] = particles[index];
            new_particles[m].weight =
                1.0f / PARTICLE_QUANTITY; // Reset weights uniformly
        }

        return new_particles;
    }

    void resampleParticles()
    {
        // Perform systematic resampling
        std::vector<Particle> new_particles = weightedResample(particles);

        for (auto &particle : new_particles)
        {
            particle.pose.x += noise_x(gen);
            particle.pose.y += noise_y(gen);
            particle.pose.theta += noise_theta(gen);

            // Normalize theta to stay within 0-360 degrees
            particle.pose.theta = fmod(particle.pose.theta, 360.0f);
            if (particle.pose.theta < 0)
            {
                particle.pose.theta += 360.0f;
            }
        }
        particles = new_particles;
    }

    lemlib::Pose getEstimatedPose()
    {
        lemlib::Pose rawEstimated(0, 0, 0);
        float total_weight = 0.0f;

        for (const auto &particle : particles)
        {
            rawEstimated.x += particle.weight * particle.pose.x;
            rawEstimated.y += particle.weight * particle.pose.y;
            rawEstimated.theta += particle.weight * particle.pose.theta;
            total_weight += particle.weight;
        }

        if (total_weight > 0)
        {
            rawEstimated.x /= total_weight;
            rawEstimated.y /= total_weight;
            rawEstimated.theta /= total_weight;
        }

        // Apply low-pass filter
        filteredPose.x =
            FILTER_ALPHA * rawEstimated.x + (1 - FILTER_ALPHA) * filteredPose.x; // Apply filter to x {{ Applied filter to x }}
        filteredPose.y =
            FILTER_ALPHA * rawEstimated.y + (1 - FILTER_ALPHA) * filteredPose.y; // Apply filter to y {{ Applied filter to y }}

        // Filter theta with angle wrapping
        float theta_diff = rawEstimated.theta - filteredPose.theta;
        if (theta_diff > 180)
            theta_diff -= 360;
        if (theta_diff < -180)
            theta_diff += 360;
        filteredPose.theta += FILTER_ALPHA * theta_diff; // Apply filter to theta {{ Applied filter to theta }}

        // Normalize theta
        while (filteredPose.theta > 360)
            filteredPose.theta -= 360;
        while (filteredPose.theta < 0)
            filteredPose.theta += 360;

        // //! Apply odometry trust factor (do we need this?)
        // lemlib::Pose blendedPose(0, 0, 0);                                                                     // {{ Added blendedPose variable }}
        // blendedPose.x = ODOMETRY_TRUST_FACTOR * lastOdomPose.x + (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.x; // {{ Blended x with odometry }}
        // blendedPose.y = ODOMETRY_TRUST_FACTOR * lastOdomPose.y + (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.y; // {{ Blended y with odometry }}

        // // Handle theta blending with angle wrapping
        // theta_diff = filteredPose.theta - lastOdomPose.theta; // {{ Recalculate theta_diff for blending }}
        // if (theta_diff > 180)
        //     theta_diff -= 360;
        // if (theta_diff < -180)
        //     theta_diff += 360;
        // blendedPose.theta = lastOdomPose.theta + (1 - ODOMETRY_TRUST_FACTOR) * theta_diff; // {{ Blended theta with odometry }}

        return filteredPose;
    }


    void mclTask(void *param)
    {

        // Initialize with current pose
        initializeParticles(Chassis::getChassis().getPose());
        lastOdomPose = Chassis::getChassis().getPose(); // Record initial pose for delta calculation
        lastUpdateTime = pros::millis();

        int resampleCounter = 0; // Only resample every 3rd update

        lemlib::Pose currentOdomPose(0,0,0);
        lemlib::Pose motionDelta(0,0,0);

        while (mclRunning)
        {
            // Get distance readings and filter unreliable values
            float north = useNorthSensor ? dNorth.get() / 25.4 : -1;
            float south = useSouthSensor ? dSouth.get() / 25.4 : -1;
            float east = useEastSensor ? dEast.get() / 25.4 : -1;
            float west = useWestSensor ? dWest.get() / 25.4 : -1;

            // Apply confidence and size filters
            int north_conf = dNorth.get_confidence();
            int south_conf = dSouth.get_confidence();
            int east_conf = dEast.get_confidence();
            int west_conf = dWest.get_confidence();

            int north_size = dNorth.get_object_size();
            int south_size = dSouth.get_object_size();
            int east_size = dEast.get_object_size();
            int west_size = dWest.get_object_size();

            const int MIN_CONFIDENCE = 45;
            const int MIN_OBJECT_SIZE = 50;
            const int MAX_OBJECT_SIZE = 401;

            if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE ||
                north_size > MAX_OBJECT_SIZE || north >= 9999 || north > 210)
                north = -1;
            if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE ||
                south_size > MAX_OBJECT_SIZE || south >= 9999 || south > 210)
                south = -1;
            if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE ||
                east_size > MAX_OBJECT_SIZE || east >= 9999 || east > 210)
                east = -1;
            if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE ||
                west_size > MAX_OBJECT_SIZE || west >= 9999 || west > 210)
                west = -1;

            if (!useNorthSensor)
                north = -1;
            if (!useSouthSensor)
                south = -1;
            if (!useEastSensor)
                east = -1;
            if (!useWestSensor)
                west = -1;

            int currentTime = pros::millis();

            // Get current odometry pose and calculate delta
            currentOdomPose = Chassis::getChassis().getPose();
            
            motionDelta.x = currentOdomPose.x - lastOdomPose.x;
            motionDelta.y = currentOdomPose.y - lastOdomPose.y;
            motionDelta.theta = currentOdomPose.theta - lastOdomPose.theta;
            
            // Normalize Theta Delta
            if(motionDelta.theta > 180) motionDelta.theta -= 360;
            if(motionDelta.theta < -180) motionDelta.theta += 360;

            float motion_magnitude = std::sqrt(motionDelta.x * motionDelta.x + 
                                             motionDelta.y * motionDelta.y);

            // Motion update
            if ((motion_magnitude > MOTION_NOISE_THRESHOLD) || currentTime - lastUpdateTime >= UPDATE_INTERVAL)
            {

                motionUpdate(motionDelta);

                // Measurement update
                measurementUpdate(north, south, east, west);

                // Only resample periodically to prevent particle depletion
                resampleCounter++;
                if (resampleCounter >= RESAMPLING_INTERVAL)
                {
                    resampleParticles();
                    resampleCounter = 0;
                }

                // Get estimated pose
                lemlib::Pose estimatedPose = getEstimatedPose();

                // Add debugging
                printf("Estimated: (%.2f, %.2f, %.2f) | Motion: (%.4f, %.4f, %.4f)\n",
                       estimatedPose.x, estimatedPose.y, estimatedPose.theta,
                       motionDelta.x, motionDelta.y, motionDelta.theta);

                // Update chassis position
                Chassis::getChassis().setPose(estimatedPose.x, estimatedPose.y,
                                              estimatedPose.theta);

                lastOdomPose = estimatedPose; // Update last recorded pose

                lastUpdateTime = currentTime;

            }

            pros::delay(MCL_DELAY);

        }
    }

    void startMCL(lemlib::Pose initialPose)
    {
        Chassis::getChassis().setPose(initialPose);
        mclRunning = true;
        mclTaskHandle = new pros::Task(mclTask, nullptr, "MCL Task");
    }

    void stopMCL()
    {
        if (mclTaskHandle != nullptr)
        {
            mclRunning = false;
            pros::delay(MCL_DELAY * 2); // Give task time to stop
            delete mclTaskHandle;
            mclTaskHandle = nullptr;
        }
    }

}