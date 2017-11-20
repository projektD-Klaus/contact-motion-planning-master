#ifndef _RL_PLAN_CERRTEXTENSIONS_H_
#define _RL_PLAN_CERRTEXTENSIONS_H_

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>

#include <rl/sg/solid/Scene.h>
#include <rl/plan/Model.h>

#include <Eigen/Eigenvalues>

namespace rl
{
namespace plan
{
class Gaussian {
public:

  Gaussian()
  {
  }

  Gaussian(const ::std::vector<::rl::math::Vector>& values)
  {
    this->init(values);
  }

  Gaussian(const ::rl::math::Matrix& values)
  {
    this->init(values);
  }


  ::rl::math::Real mahalanobis(::rl::math::Vector& x)
  {
    return sqrt((x - this->mean).transpose() * this->covariance.inverse() * (x - this->mean));
  }

  ::rl::math::Matrix eigenvectors()
  {
    ::Eigen::EigenSolver<::rl::math::Matrix> eig(this->covariance);
    return eig.eigenvectors().real();
  }

  ::rl::math::Vector eigenvalues()
  {
    ::Eigen::EigenSolver<::rl::math::Matrix> eig(this->covariance);
    return eig.eigenvalues().real();
  }

  ::rl::math::Vector mean;
  ::rl::math::Matrix covariance;

  void init(const ::std::vector<::rl::math::Vector>& values)
  {
    ::rl::math::Matrix p;
    p.resize(values.size(), values[0].size());
    for (int i = 0; i < values.size(); ++i)
    {
      p.row(i) = values[i];
    }
    this->init(p);
  }

  void init(const ::rl::math::Matrix& values)
  {
    this->mean = values.colwise().mean();
    if (values.rows() == 1)
    {
      // just one particle
      this->covariance = ::rl::math::Matrix::Zero(values.cols(), values.cols());
      return;
    }
    // substract mean
    ::rl::math::Matrix centered = values.rowwise() - this->mean.transpose();
    // calculate sample covariance
    this->covariance = centered.transpose() * centered / (values.rows()-1);
  }
};

class Contact
{
public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact(){}

  Contact(::rl::math::Vector3& p, ::rl::math::Vector3& n, std::string& s_robot, std::string& s_env) :
    point(p),
    normal_env(n),
    shape_robot(s_robot),
    shape_env(s_env)
  {}

  ::rl::math::Vector3 point;
  std::string shape_robot;
  std::string shape_env;
  ::rl::math::Vector3 normal_env; //Normal vector on environment
};

class Particle
{
public:
  Particle(){}

  Particle(::rl::math::Vector& q, ::rl::sg::CollisionMap &c) :
    config(q),
    contacts(c)
  {
  }

public:
  Particle(::rl::math::Vector& q) :
    config(q)
  {
  }

  std::vector <int> planeIDs; //< the planes that this particle is in contact with
  ::rl::math::Vector config;
  //Todo: this should be moved into belief state
  ::rl::sg::CollisionMap contacts;
};

class BeliefState
{
public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BeliefState(){}

  BeliefState(const ::std::vector<Particle>& particles, rl::plan::Model* model, const ::rl::math::Vector3 normal = ::rl::math::Vector3()) :
    gen(42),
    particles(particles),
    slidingNormal(normal),
    actionType(0)
  {
    this->init(particles, model);
  }

  void sampleConfiguration(::rl::math::Vector& q)
  {
    assert(q.size() == this->dims);

    for (int i = 0; i < this->dims; ++i)
    {
      q[i] = this->configSampleDistributions[i](this->gen);
    }
  }


  void sampleConfigurationFromParticle(::rl::math::Vector& q, int id = -1)
  {
    assert(q.size() == this->dims);

    if(id==-1)
    {
      boost::random::uniform_int_distribution<> particleDistr(0, particles.size()-1);
      id = particleDistr(this->gen);
    }

    q = this->particles[id%particles.size()].config;
  }


  Gaussian configGaussian()
  {
    return this->configDistr;
  }

  ::rl::math::Vector configMean()
  {
    return this->configDistr.mean;
  }

  ::rl::math::Matrix configCovariance()
  {
    return this->configDistr.covariance;
  }

  Gaussian eeGaussian()
  {
    return this->eePosDistr;
  }

  ::rl::math::Vector eeMean()
  {
    return this->eePosDistr.mean;
  }

  ::rl::math::Matrix eeCovariance()
  {
    return this->eePosDistr.covariance;
  }


  bool isInCollision()
  {
    return (this->particles[0].contacts.size() != 0);
  }

  bool isSlidingMove()
  {
    return (this->slidingNormal.norm()>0.1);
  }

  ::rl::math::Vector3 getSlidingNormal()
  {
    return this->slidingNormal;
  }

  void setActionType (int actionType_)
  {
    actionType = actionType_;
  }

  int getActionType () const
  {
    return actionType;
  }

  const ::std::vector<Particle>& getParticles()
  {
    return particles;
  }

private:
  void init(const ::std::vector<Particle>& particles, rl::plan::Model* model)
  {
    std::vector<::rl::math::Vector> q_vec;
    std::vector<::rl::math::Vector> ee_vec;
    for(int i=0; i<particles.size(); i++)
    {
      q_vec.push_back(particles[i].config);

      model->setPosition(particles[i].config);
      model->updateFrames();
      rl::math::Transform t = model->forwardPosition();
      rl::math::Vector p(3);
      p(0)=t(0,3);
      p(1)=t(1,3);
      p(2)=t(2,3);
      ee_vec.push_back(p);

    }
    configDistr.init(q_vec);
    eePosDistr.init(ee_vec);

    this->dims = particles[0].config.size();
    for (int i = 0; i < this->dims; ++i)
    {
      ::rl::math::Real mean = this->configDistr.mean[i];
      ::rl::math::Real std_dev = sqrt(this->configDistr.covariance(i,i));
      configSampleDistributions.push_back(boost::random::normal_distribution<>(mean, std_dev));
    }

    for (int i = 0; i < 3; ++i)
    {
      ::rl::math::Real mean = this->eePosDistr.mean[i];
      ::rl::math::Real std_dev = sqrt(this->eePosDistr.covariance(i,i));
      eePosSampleDistributions.push_back(boost::random::normal_distribution<>(mean, std_dev));
    }
  }

  int actionType; //< The action that led to the creation of this node (one of CERRT actions)
  ::std::vector<Particle> particles;
  ::rl::math::Vector3 slidingNormal;
  Gaussian configDistr;
  Gaussian eePosDistr;
  boost::random::mt19937 gen;
  ::std::vector<boost::random::normal_distribution<> > configSampleDistributions;
  ::std::vector<boost::random::normal_distribution<> > eePosSampleDistributions;
  int dims;
};
}
}

#endif // _RL_PLAN_CERRTEXTENSIONS_H_
