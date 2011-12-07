#ifndef GRID_CALIBRATOR_H
#define GRID_CALIBRATOR_H

#include <eigen3/Eigen/Core>

#include <vector>

namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////////
    /// Compute extrinsics
    bool compute_extrinsics( const CameraModel& cameraModel,
                             const Eigen::MatrixXd& mP3D,
                             const Eigen::MatrixXd& mP2D,
                             Eigen::Matrix3d& mR,
                             Eigen::Vector3d& mt );

    ////////////////////////////////////////////////////////////////////////////////
    /// Compute extrinsics
    bool compute_extrinsics_non_lin
        ( const CameraModel& cameraModel,
          const Eigen::MatrixXd& mP3D,
          const Eigen::MatrixXd& mP2D,
          Eigen::Matrix3d& mR,
          Eigen::Vector3d& mt );

    ////////////////////////////////////////////////////////////////////////////////
    void compute_update( const std::vector<Eigen::Matrix3d>& vR, 
                         const std::vector<Eigen::Vector3d>& vt,
                         const CameraModel& cameraModel,
                         const std::vector<Eigen::MatrixXd>& vImagePoints,
                         const Eigen::MatrixXd& m_mGrid,
                         const double dStabiliser, ///<Input: Levenberg-Marquardt identity matrix multiplier
                         Eigen::VectorXd& vUpdate, ///<Output: least-squares parameter update
                         double& dRMS,     ///<Output: reprojection error
                         double& dRMSGain, ///<Output: predicted gain from the linearisation
                         double& dGradMagn,///<Output: magnitude of the least-squares Jacobian
                         bool bFull ); 

    //////////////////////////////////////////////////////////////////////////////// 
    void compute_update_extrinsic
        ( const std::vector<Eigen::Matrix3d>& vR, 
          const std::vector<Eigen::Vector3d>& vt,
          const CameraModel& cameraModel, 
          const std::vector<Eigen::MatrixXd>& vImagePoints,
          const Eigen::MatrixXd& mGrid,
          const double dStabiliser,
          Eigen::VectorXd& vUpdate, ///<Output: least-squares parameter update
          double& dRMS,     ///<Output: reprojection error
          double& dRMSGain, ///<Output: predicted gain from the linearisation
          double& dGradMagn ///<Output: magnitude of the least-squares Jacobian
          );

    ////////////////////////////////////////////////////////////////////////////////
    void apply_update( const Eigen::VectorXd& vUpdate,
                       std::vector<Eigen::Matrix3d>& vR, 
                       std::vector<Eigen::Vector3d>& vt
                       );

    ////////////////////////////////////////////////////////////////////////////////
    void apply_update( const Eigen::VectorXd& vUpdate,
                       std::vector<Eigen::Matrix3d>& vR, 
                       std::vector<Eigen::Vector3d>& vt,
                       CameraModel& cameraModel
                       );

    ////////////////////////////////////////////////////////////////////////////////
    void print_values( const CameraModel& cameraModel );

    ////////////////////////////////////////////////////////////////////////////////
    class GridCalibrator {
    public:
        GridCalibrator( const std::string& sCameraType,
                        const unsigned int nImageWidth,
                        const unsigned int nImageHeight,
                        const unsigned int nBoardWidth,
                        const unsigned int nBoardHeight
                        );
        
        ////////////////////////////////////////////////////////////////////////
        /// Add a new view with a set of 2D measurements
        /// (internally the extrinsic will be initialised)
        void add_view( const Eigen::MatrixXd& mP2D );

        ////////////////////////////////////////////////////////////////////////
        /// Make one full update step.
        /// Returns false if the error increases.
        bool iterate( double& dRMS );

        ////////////////////////////////////////////////////////////////////////
        /// Make a full minimisation
        double minimise();

        ////////////////////////////////////////////////////////////////////////
        int get_num_views() { return m_vR.size(); }

        ////////////////////////////////////////////////////////////////////////
        bool save( const std::string& sFileName ); 

        ////////////////////////////////////////////////////////////////////////
        void print();
        
        ////////////////////////////////////////////////////////////////////////
        CameraModel* get_camera_copy();
    private:
        CameraModel m_CameraModel;
        unsigned int m_nImageWidth, m_nImageHeight;
        std::vector<Eigen::Matrix3d> m_vR;
        std::vector<Eigen::Vector3d> m_vt;
        Eigen::MatrixXd m_mGrid;
        std::vector<Eigen::MatrixXd> m_vImagePoints;
        double m_dPrevRMS;        
    };
}

#endif
