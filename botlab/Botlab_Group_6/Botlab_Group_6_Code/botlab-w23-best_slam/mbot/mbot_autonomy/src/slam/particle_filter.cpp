#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <common_utils/geometric/angle_functions.hpp>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleweight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    for (auto & p :posterior_) {
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleweight;
    }

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleweight = 1.0 / kNumParticles_;
    ParticleList randomParticles;

    // uniformly set particle locations across map
    int mapwidth = map.widthInCells();
    int mapheight = map.heightInCells();
    int totcells = mapwidth * mapheight;
    int ratio = kNumParticles_ / totcells;
    int pwidth = ratio * mapwidth;
    int pheight = ratio * mapheight;

    for (int x = 0; x < pwidth; ++x) { // particle in particle grid width
        for (int y = 0; y < pheight; ++y) { // 
            mbot_lcm_msgs::particle_t newparticle;
            newparticle.pose.x = x / ratio;
            newparticle.pose.y = y / ratio;
            newparticle.pose.theta = 0;
            // newparticle.pose.utime = utime; // not sure what time to set them at
            newparticle.parent_pose = newparticle.pose;
            newparticle.weight = sampleweight;
            randomParticles.push_back(newparticle);
        }
    }
    posterior_ = randomParticles;
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved){
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
        // posteriorPose_ = computeParticlesAverage(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        // auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    ParticleList prior = posterior_;
    ParticleList resampled;
    int M = kNumParticles_;
    double sampleWeight = 1.0 / kNumParticles_; // M^-1

    srand(time(nullptr));
    double r = (rand() / (RAND_MAX+0.0)) * (1.0/M); // [0,M^-1] random number from 0 to M^-1
    float c = prior[0].weight;
    int i = 1;
   
    for (int m = 1; m <= M; m++) {
        float u = r + (m - 1.0) * sampleWeight;
        while(u > c) {
            i += 1;
            c += prior[i - 1].weight;
        }
        resampled.push_back(prior[i - 1]);
    }

    return resampled;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for (auto & p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    // ParticleList posterior;
    std::vector<mbot_lcm_msgs::particle_t> posterior;
    double sumWeights = 0.0;
    for (auto& p : proposal) {
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for (auto& p : posterior) {
        p.weight /= sumWeights;
    }

    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    //Weighted average of pose???
    mbot_lcm_msgs::pose_xyt_t pose;
    //return computeParticlesAverage(posterior);

    double xMean = 0.0, yMean = 0.0, cosThetaMean = 0.0, sinThetaMean = 0.0;

    for (auto& p : posterior) {
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
    }
    
    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);

    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    //TODO: Weights??? weighted average?
    float sum_x = 0, sum_y = 0, sum_sin = 0, sum_cos = 0, sum_weights = 0;
    for (auto & p : particles_to_average) {
        sum_x += (p.pose.x * p.weight);
        sum_y += (p.pose.y * p.weight);
        sum_sin += (p.weight * std::sin(p.pose.theta));
        sum_cos += (p.weight * std::cos(p.pose.theta));
        sum_weights += p.weight;
    }
    avg_pose.x = sum_x/sum_weights;
    avg_pose.y = sum_y/sum_weights;
    avg_pose.theta = atan2(sum_sin, sum_cos);
    return avg_pose;
}
