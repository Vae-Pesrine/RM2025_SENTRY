#include <fstream>
#include <iostream>
#include <math.h>
#include <boost/gil.hpp>
#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>
#include <sdf/Element.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "collision_map_request.pb.h"
namespace gazebo
{
using CollisionMapRequestPtr = const boost::shared_ptr<const collision_map_creator_msgs::msgs::CollisionMapRequest>;

class CollisionMapCreator: public WorldPlugin
{
    transport::NodePtr node;
    transport::PublisherPtr imagePub;
    transport::SubscriberPtr commandSubscriber;
    physics::WorldPtr world;

public:
    //Rewrite the virtual functions to initialize the plugin
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        node = transport::NodePtr(new transport::Node());
        world = _parent;
        node->Init(world->Name());
        std::cout << "Subscribing to: " << "~/collision_map/command" << std::endl;
        commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
        imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
    }

    void create(CollisionMapRequestPtr &msg)
    {
        std::cout << "Receiver message" << std::endl;

        std::cout << "Creating collision map with corners at (" <<
                     msg->upperleft().x() << ", " << msg->upperleft().y() << "), (" <<
                     msg->upperright().x() << ", " << msg->upperright().y() << "), (" <<
                     msg->lowerleft().x() << ", " << msg->lowerleft().y() << "), (" <<
                     msg->lowerright().x() << ", " << msg->lowerright().y() << 
                     ") with collision projected from z = " << msg->height() << 
                     "\nresolution() = " << msg->resolution() << "m\n" << 
                     "Occupied spaces will be filled with: " << msg->threshold() << std::endl;

        double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
        double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
        double mag_vertical = std::hypot(dX_vertical, dY_vertical);                     
        dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
        dY_vertical = msg->resolution() * dY_vertical / mag_vertical;

        double dX_horizontal = msg->upperright().x() - msg->upperleft().x();
        double dY_horizontal = msg->upperright().y() - msg->upperleft().y();
        double mag_horizontal = std::hypot(dX_horizontal, dY_horizontal);
        dX_horizontal = msg->resolution() * dX_horizontal / mag_horizontal;
        dY_horizontal = msg->resolution() * dY_horizontal / mag_horizontal;

        int count_vertical = mag_vertical / msg->resolution();
        int count_horizontal = mag_horizontal / msg->resolution();

        if(count_horizontal == 0 || count_vertical == 0){
            std::cout << "Image has a zero dimensions, check coordinates" << std::endl;
            return;
        }

        double x, y;
        boost::gil::gray8_pixel_t fill(255 - msg->threshold());
        boost::gil::gray8_pixel_t blank(255);
        boost::gil::gray8_image_t image(count_horizontal, count_vertical);

        double dist;
        std::string entityName;
        ignition::math::Vector3d start, end;
        start.Z(msg->height());
        end.Z(0.5);

        gazebo::physics::PhysicsEnginePtr engine = world->Physics();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

        std::cout << "Rasterizing model and checking collisions" << std::endl;
        boost::gil::fill_pixels(image._view, blank);

        for(int i = 0; i < count_vertical; ++i){
            // std::cout << "Precent complete: " << i * 100.0 / count_vertical << std::endl;
            x = i * dX_vertical + msg->lowerleft().x();
            y = i * dY_vertical + msg->lowerleft().y();
            for(int j = 0; j < count_horizontal; ++j){
                x += dX_horizontal;
                y += dY_horizontal;

                start.X(x);
                end.X(x);
                start.Y(y);
                end.Y(y);
                ray->SetPoints(start, end);
                ray->GetIntersection(dist, entityName);
                if(!entityName.empty()){
                    image._view(i, j) = fill;
                }
            }
        }

        std::cout << "Completed calculations, wirting to image" << std::endl;
        if(!msg->filename().empty()){
            boost::gil::gray8_view_t view = image._view;
            pgm_write_view(msg->filename(), view);
        }
        std::cout << "Output location: " << msg->filename() << std::endl;
    }

    void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
    {
        std::cout << "running" << std::endl;
        int h = view.height();
        int w = view.width();

        std::ofstream ofs;
        ofs.open(filename + ".pgm");
        ofs << "P2" << '\n';
        ofs << w << ' ' << h << '\n';
        ofs << 255 << '\n';
        for(int y = 0; y < h; ++y){
            for(int x = 0; x < w; ++x){
                ofs << static_cast<int>(view(x, y)[0]) << ' ';
            }
            ofs << '\n';
        }
        ofs.close();
    }
};

GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
} // namespace gazebo