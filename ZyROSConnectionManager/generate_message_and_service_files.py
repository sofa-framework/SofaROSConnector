#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import argparse
import os
import fnmatch

logger_messages = logging.getLogger('ROSMessageBindingGeneration')
logger_services = logging.getLogger('ROSServiceBindingGeneration')

logger_main = logging.getLogger('main')


class ROSBindingGenerator(object):

    def __init__(self):
        self.logger = logging.getLogger('ROSBindingGenerator')
        self.ros_install_prefix = os.getenv('ROS_ROOT')
        self.ros_include_dir = os.path.join(self.ros_install_prefix, '..', '..', 'include')

    def find_matching_headers(self, ros_definitions, exact_match=True):
        self.logger.info('Generating ROS message binding sources. Message definitions passed: ' + str(len(ros_definitions)))
        matching_files_per_ros_definition = {}

        for ros_message_type in ros_definitions:
            message_type_parts = ros_message_type.split('/')
            if len(message_type_parts) == 2:
                message_type_package = message_type_parts[0]
                message_type_name = message_type_parts[1]

                self.logger.debug('Message \'' + message_type_name + '\' from package \'' + message_type_package + '\'')
                message_package_include_dir = os.path.join(self.ros_include_dir, message_type_package)
                if os.path.exists(message_package_include_dir):
                    self.logger.debug('Include directory for message type exists: ' + message_package_include_dir)
                    matching_header_files = []
                    for file in os.listdir(message_package_include_dir):
                        file_name_to_match = ''
                        if not exact_match:
                            file_name_to_match = message_type_name + '*.h'
                        else:
                            file_name_to_match = message_type_name + '.h'

                        if fnmatch.fnmatch(file, file_name_to_match):
                            self.logger.debug('Found include file matching message_type name: ' + file)
                            matching_header_files.append(os.path.join(message_type_package, file))

                    self.logger.debug('Matching header files found: ' + str(len(matching_header_files)))
                    matching_files_per_ros_definition[ros_message_type] = matching_header_files

        return matching_files_per_ros_definition

    def generate_binding_sources(self):
        pass


class ROSMessageBindingGenerator(ROSBindingGenerator):

    def __init__(self, base_directory, ros_message_types = []):
        ROSBindingGenerator.__init__(self)
        self.__ros_message_types = ros_message_types
        self.__base_directory = base_directory
        self.__script_directory = os.path.dirname(os.path.realpath(__file__))
        self.__ros_message_subscribers_generated_h_name = 'ZyROS_MessageType_Instantiations_Subscribers.h'
        self.__ros_message_subscribers_generated_cpp_name = 'ZyROSConnectorTopicSubscriber.cpp'
        self.__ros_message_publishers_generated_h_name = 'ZyROS_MessageType_Instantiations_Publishers.h'
        self.__ros_message_publishers_generated_cpp_name = 'ZyROSConnectorTopicPublisher.cpp'

        self.__excluded_message_names = [] # These break plugin loading when included directly under ROS kinetic

        self.__message_subscriber_cpp_file_static = """
#include "ZyROSConnectorTopicSubscriber.inl"

using namespace Zyklio::ROSConnector;

SOFA_DECL_CLASS(ZyROSConnectorTopicSubscriberIface)

ZyROSListener::ZyROSListener()
    : m_uuid(boost::uuids::random_generator()())
    , messageType("")
{

}

ZyROSListener::ZyROSListener(const ZyROSListener& other)
{
        if (this != &other)
        {
                m_uuid = other.m_uuid;
                m_rosTopic = other.m_rosTopic;
        }
}

ZyROSListener& ZyROSListener::operator=(const ZyROSListener& other)
{
        if (this != &other)
        {
                m_uuid = other.m_uuid;
                m_rosTopic = other.m_rosTopic;
        }
        return *this;
}

void ZyROSConnectorTopicSubscriberIface::onMessageReceived()
{
        m_sig();
}
        """

        self.__message_publisher_cpp_file_static =  """
#include "ZyROSConnectorTopicPublisher.inl"

using namespace Zyklio::ROSConnector;

ZyROSPublisher::ZyROSPublisher() : m_uuid(boost::uuids::random_generator()())
{

}

ZyROSPublisher::ZyROSPublisher(const ZyROSPublisher& other)
{
    if (this != &other)
    {
        m_uuid = other.m_uuid;
        messageType = other.messageType;
    }
}

ZyROSPublisher& ZyROSPublisher::operator=(const ZyROSPublisher& other)
{
    if (this != &other)
    {
        m_uuid = other.m_uuid;
        messageType = other.messageType;
    }
    return *this;
}

template <class MessageType>
void ZyROSConnectorTopicPublisher<MessageType>::publishMessageQueue()
{
    boost::mutex::scoped_lock lock(m_mutex);

    if (!m_messageQueue.empty())
    {
        msg_info("ZyROSConnectorTopicPublisher") << "publishMessageQueue of size " << m_messageQueue.size();
        while (!m_messageQueue.empty())
        {
            MessageType& msg = m_messageQueue.back();
            m_publisher.publish(msg);
            m_messageQueue.pop_back();
        }
    }
    lock.unlock();
}
        """

    def generate_ros_message_subscriber_files(self):
        self.logger.debug('Generating header and CPP for ROS message subscriber instances.')
        header_files_per_message_type = self.find_matching_headers(self.__ros_message_types, True)
        if len(header_files_per_message_type.keys()) > 0:
            message_source_file_path = os.path.join(self.__base_directory, self.__ros_message_subscribers_generated_h_name)
            self.logger.debug('Writing subscriber header file at: ' + message_source_file_path)
            message_source_file = open(message_source_file_path, 'w+')

            message_source_file.write('/***********************************************************************\n')
            message_source_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_source_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_source_file.write('Please do not modify it by hand.\n')
            message_source_file.write('The contents will be overwritten and re-generated.\n')
            message_source_file.write('************************************************************************/\n')
            message_source_file.write('\n\n')

            message_source_file.write('#include <ZyROSConnectorTopicSubscriber.h>\n')
            message_source_file.write('#include <ZyROSConnectorTopicPublisher.h>\n')

            message_source_file.write('\n')

            for message_type in sorted(header_files_per_message_type.keys()):
                self.logger.debug('Matching header files for message type \'' + message_type + '\': ' + str(len(header_files_per_message_type[message_type])))

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    for header_file in header_files_per_message_type[message_type]:
                        message_source_file.write('#include <' + header_file + '>\n')

            message_source_file.write('\n\n')

            message_source_file.write('#include <boost/shared_ptr.hpp>\n')

            message_source_file.write('namespace Zyklio\n')
            message_source_file.write('{\n')
            message_source_file.write('\tnamespace ROSConnector\n')
            message_source_file.write('\t{\n')

            message_source_file.write('\t\tclass ZyROSConnectorMessageSubscriberFactory\n')
            message_source_file.write('\t\t{\n')
            message_source_file.write('\t\tpublic:\n')
            message_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSListener> createTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType);\n')
            message_source_file.write('\t\t};\n')

            message_source_file.write('\t}\n')
            message_source_file.write('}\n')

            message_source_file.close()

            message_cpp_file_path = os.path.join(self.__script_directory, '..', 'ZyROSConnector', self.__ros_message_subscribers_generated_cpp_name)
            self.logger.info('Writing CPP file for messages: ' + message_cpp_file_path)
            message_cpp_file = open(message_cpp_file_path, 'w+')

            message_cpp_file.write('/***********************************************************************\n')
            message_cpp_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_cpp_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_cpp_file.write('Please do not modify it by hand.\n')
            message_cpp_file.write('The contents will be overwritten and re-generated.\n')
            message_cpp_file.write('************************************************************************/\n')
            message_cpp_file.write('\n\n')

            message_cpp_file.write(self.__message_subscriber_cpp_file_static + '\n\n')

            message_cpp_file.write('#include <' + self.__ros_message_subscribers_generated_h_name + '>\n')
            message_cpp_file.write('\n\n')
            message_cpp_file.write('using namespace Zyklio::ROSConnector;\n')
            for message_type in sorted(header_files_per_message_type.keys()):
                self.logger.debug('Matching header files for message type \'' + message_type + '\': ' + str(len(header_files_per_message_type[message_type])))

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    message_cpp_file.write('// Publisher and subscriber proxy class instantiation for ROS message type: ' + message_type + '\n')
                    for header_file in header_files_per_message_type[message_type]:
                        message_cpp_type = message_type.replace('/', '::')

                        message_cpp_file.write('template class ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>;\n')
                        message_cpp_file.write('\n')

            message_cpp_file.write('boost::shared_ptr<ZyROSListener> ZyROSConnectorMessageSubscriberFactory::createTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)\n')
            message_cpp_file.write('{\n')

            message_cpp_file.write('\tbool supported = false;\n')
            message_cpp_file.write('\tboost::shared_ptr<ZyROSListener> topicListener;\n')

            for message_type in sorted(header_files_per_message_type.keys()):

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    message_cpp_file.write('\t// Subscriber instance for ROS message type: ' + message_type + '\n')
                    message_cpp_type = message_type.replace('/', '::')
                    message_cpp_file.write('\tif (messageType == "' + message_cpp_type + '")\n')
                    message_cpp_file.write('\t{\n')
                    message_cpp_file.write('\t\tsupported = true;\n')
                    # message_cpp_file.write('\t\tconst boost::shared_ptr<ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>> tmp();\n')
                    message_cpp_file.write('\t\ttopicListener.reset(new ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>(rosNode, topicURI, 50, true));\n')
                    message_cpp_file.write('\t}\n')


            message_cpp_file.write('\tif (supported)\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_info("ZyROSConnectorMessageSubscriberFactory") << "ROS message type supported: " << messageType;\n')
            message_cpp_file.write('\t}\n')
            message_cpp_file.write('\telse\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_warning("ZyROSConnectorMessageSubscriberFactory") << "ROS message type NOT supported: " << messageType;\n')
            message_cpp_file.write('\t}')

            message_cpp_file.write('\treturn topicListener;\n')

            message_cpp_file.write('}\n\n')

            message_cpp_file.close()

    def generate_ros_message_publisher_files(self):
        self.logger.debug('Generating header and CPP for ROS message publisher instances.')
        header_files_per_message_type = self.find_matching_headers(self.__ros_message_types, True)
        if len(header_files_per_message_type.keys()) > 0:
            message_source_file_path = os.path.join(self.__base_directory, self.__ros_message_publishers_generated_h_name)
            self.logger.debug('Writing publisher header file at: ' + message_source_file_path)
            message_source_file = open(message_source_file_path, 'w+')

            message_source_file.write('/***********************************************************************\n')
            message_source_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_source_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_source_file.write('Please do not modify it by hand.\n')
            message_source_file.write('The contents will be overwritten and re-generated.\n')
            message_source_file.write('************************************************************************/\n')
            message_source_file.write('\n\n')

            message_source_file.write('#include <ZyROSConnectorTopicPublisher.h>\n')

            message_source_file.write('\n')

            for message_type in sorted(header_files_per_message_type.keys()):
                self.logger.debug('Matching header files for message type \'' + message_type + '\': ' + str(len(header_files_per_message_type[message_type])))

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    for header_file in header_files_per_message_type[message_type]:
                        message_source_file.write('#include <' + header_file + '>\n')

            message_source_file.write('#include <boost/shared_ptr.hpp>\n')

            message_source_file.write('namespace Zyklio\n')
            message_source_file.write('{\n')
            message_source_file.write('\tnamespace ROSConnector\n')
            message_source_file.write('\t{\n')

            message_source_file.write('\t\tclass ZyROSConnectorMessagePublisherFactory\n')
            message_source_file.write('\t\t{\n')
            message_source_file.write('\t\tpublic:\n')
            message_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSPublisher> createTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType);\n')
            message_source_file.write('\t\t};\n')

            message_source_file.write('\t}\n')
            message_source_file.write('}\n')

            message_source_file.close()

            message_cpp_file_path = os.path.join(self.__script_directory, '..', 'ZyROSConnector', self.__ros_message_publishers_generated_cpp_name)
            self.logger.info('Writing CPP file for messages: ' + message_cpp_file_path)
            message_cpp_file = open(message_cpp_file_path, 'w+')

            message_cpp_file.write('/***********************************************************************\n')
            message_cpp_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_cpp_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_cpp_file.write('Please do not modify it by hand.\n')
            message_cpp_file.write('The contents will be overwritten and re-generated.\n')
            message_cpp_file.write('************************************************************************/\n')
            message_cpp_file.write('\n\n')

            message_cpp_file.write(self.__message_publisher_cpp_file_static + '\n\n')

            message_cpp_file.write('#include <' + self.__ros_message_publishers_generated_h_name + '>\n')
            message_cpp_file.write('\n\n')
            message_cpp_file.write('using namespace Zyklio::ROSConnector;\n')
            for message_type in sorted(header_files_per_message_type.keys()):
                self.logger.debug('Matching header files for message type \'' + message_type + '\': ' + str(len(header_files_per_message_type[message_type])))

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    message_cpp_file.write('// Publisher and subscriber proxy class instantiation for ROS message type: ' + message_type + '\n')
                    for header_file in header_files_per_message_type[message_type]:
                        message_cpp_type = message_type.replace('/', '::')

                        message_cpp_file.write('template class ZyROSConnectorTopicPublisher<' + message_cpp_type + '>;\n')
                        message_cpp_file.write('\n')

            message_cpp_file.write('boost::shared_ptr<ZyROSPublisher> ZyROSConnectorMessagePublisherFactory::createTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)\n')
            message_cpp_file.write('{\n')

            message_cpp_file.write('\tbool supported = false;\n')
            message_cpp_file.write('\tboost::shared_ptr<ZyROSPublisher> topicPublisher;\n')

            for message_type in sorted(header_files_per_message_type.keys()):

                message_type_is_excluded = False
                for excluded_message in self.__excluded_message_names:
                    if message_type.startswith(excluded_message):
                        self.logger.warning('Excluding ROS message definition: ' + message_type)
                        message_type_is_excluded = True

                if not message_type_is_excluded:
                    message_cpp_file.write('\t// Publisher instance for ROS message type: ' + message_type + '\n')
                    message_cpp_type = message_type.replace('/', '::')
                    message_cpp_file.write('\tif (messageType == "' + message_cpp_type + '")\n')
                    message_cpp_file.write('\t{\n')
                    message_cpp_file.write('\t\tsupported = true;\n')
                    # message_cpp_file.write('\t\tconst boost::shared_ptr<ZyROSConnectorTopicPublisher<' + message_cpp_type + '>> tmp();\n')
                    message_cpp_file.write('\t\ttopicPublisher.reset(new ZyROSConnectorTopicPublisher<' + message_cpp_type + '>(rosNode, topicURI, 50));\n')
                    message_cpp_file.write('\t}\n')


            message_cpp_file.write('\tif (supported)\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_info("ZyROSConnectorMessagePublisherFactory") << "ROS message type supported: " << messageType;\n')
            message_cpp_file.write('\t}\n')
            message_cpp_file.write('\telse\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_warning("ZyROSConnectorMessagePublisherFactory") << "ROS message type NOT supported: " << messageType;\n')
            message_cpp_file.write('\t}')

            message_cpp_file.write('\treturn topicPublisher;\n')

            message_cpp_file.write('}\n\n')

            message_cpp_file.close()

    def generate_binding_sources(self):
        self.logger.info('Generating ROS message binding sources. Message definitions passed: ' + str(len(self.__ros_message_types)))
        self.generate_ros_message_subscriber_files()
        self.generate_ros_message_publisher_files()

class ROSServiceBindingGenerator(ROSBindingGenerator):

    def __init__(self, base_directory, ros_service_types = []):
        ROSBindingGenerator.__init__(self)
        self.__ros_service_types = ros_service_types
        self.__base_directory = base_directory
        self.__script_directory = os.path.dirname(os.path.realpath(__file__))
        self.__ros_service_client_generated_h_name = 'ZyROS_ServiceType_Client_Instantiations.h'
        self.__ros_service_client_generated_cpp_name = 'ZyROSConnectorServiceClient.cpp'
        self.__ros_service_server_generated_h_name = 'ZyROS_ServiceType_Server_Instantiations.h'
        self.__ros_service_server_generated_cpp_name = 'ZyROSConnectorServiceServer.cpp'

        self.__ros_service_client_cpp_source_static = """
#include "ZyROSConnectorServiceClient.inl"

using namespace Zyklio::ROSConnector;

ZyROSServiceClient::ZyROSServiceClient(): m_uuid(boost::uuids::random_generator()())
{

}
        """

        self.__ros_service_server_cpp_source_static = """

#include "ZyROSConnectorServiceServer.inl"

using namespace Zyklio::ROSConnector;

ZyROSConnectorServiceServer::ZyROSConnectorServiceServer(/*ros::NodeHandlePtr rosNode,*/ const std::string& serviceURI): m_d(NULL)
{
    m_d = new ZyROSConnectorServiceServerPrivate();
    m_d->m_serviceURI = serviceURI;
    m_d->m_rosNodeHandle.reset(new ros::NodeHandle());
    m_shutdownRequested = false;
}

ZyROSConnectorServiceServer::ZyROSConnectorServiceServer(const ZyROSConnectorServiceServer& other): m_d(NULL)
{
    m_d = new ZyROSConnectorServiceServerPrivate();
    m_d->m_serviceURI = other.m_d->m_serviceURI;
    m_d->m_rosNodeHandle.reset(new ros::NodeHandle());
    m_shutdownRequested = other.m_shutdownRequested;
}

ZyROSConnectorServiceServer::~ZyROSConnectorServiceServer()
{
    if (m_d)
    {
        if (m_d->m_rosNodeHandle)
            m_d->m_rosNodeHandle->shutdown();

        delete m_d;
        m_d = NULL;
    }
}

void ZyROSConnectorServiceServer::shutdownServer()
{
    boost::mutex::scoped_lock lock(m_mutex);
    m_shutdownRequested = true;
}

void ZyROSConnectorServiceServer::serverLoop()
{
    msg_info("ZyROSConnectorServiceServer") << "Entering serverLoop.";

    ros::Rate service_server_rate(25);
    m_serverThreadActive = true;
    while (ros::ok())
    {
        if (m_shutdownRequested)
            break;

        service_server_rate.sleep();
    }

    m_serverThreadActive = false;

    if (m_d->m_rosServer)
    {
        msg_info("ZyROSConnectorServiceServer") << "Shutting down ROS service server.";
        m_d->m_rosServer.shutdown();
    }

    if (m_d->m_rosNodeHandle)
    {
        msg_info("ZyROSConnectorServiceServer") << "Shutting down service server ROS node.";
        m_d->m_rosNodeHandle->shutdown();
    }
}

        """

        self.__excluded_service_names = [] # These break compilation when included directly under ROS kinetic

    def generate_binding_sources(self):
        self.logger.info('Generating ROS service binding sources. Service definitions passed: ' + str(len(self.__ros_service_types)))

        header_files_per_service_type = self.find_matching_headers(self.__ros_service_types, False)
        if len(header_files_per_service_type.keys()) > 0:
            service_source_file_path = os.path.join(self.__base_directory, self.__ros_service_client_generated_h_name)
            self.logger.info('Writing header file for services: ' + service_source_file_path)
            service_source_file = open(service_source_file_path, 'w+')

            service_source_file.write('/***********************************************************************\n')
            service_source_file.write('ROS service definition headers and ROS connector template instantiations.')
            service_source_file.write('This file is AUTO-GENERATED during the CMake run.')
            service_source_file.write('Please do not modify it by hand.')
            service_source_file.write('The contents will be overwritten and re-generated.')
            service_source_file.write('************************************************************************/\n')
            service_source_file.write('\n\n')

            service_source_file.write('#include <ZyROSConnectorServiceClient.h>\n')

            service_source_file.write('\n\n')

            for service_type in sorted(header_files_per_service_type.keys()):
                self.logger.debug('Matching header files for service type \'' + service_type + '\': ' + str(len(header_files_per_service_type[service_type])))

                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    for header_file in header_files_per_service_type[service_type]:
                        self.logger.debug('WRITING INCLUDE STATEMENT FOR SERVICE: ' + '#include <' + header_file + '>')
                        service_source_file.write('#include <' + header_file + '>\n')

            service_source_file.write('\n\n')
            service_source_file.write('namespace Zyklio\n')
            service_source_file.write('{\n')
            service_source_file.write('\tnamespace ROSConnector\n')
            service_source_file.write('\t{\n')

            service_source_file.write('\t\tclass ZyROSConnectorServiceClientFactory\n')
            service_source_file.write('\t\t{\n')
            service_source_file.write('\t\tpublic:\n')
            service_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSServiceClient> createServiceClient(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType);\n')
            service_source_file.write('\t\t};\n')

            service_source_file.write('\t}\n')
            service_source_file.write('}\n')

            service_source_file.close()

            service_cpp_file_path = os.path.join(self.__script_directory, '..', 'ZyROSConnector', self.__ros_service_client_generated_cpp_name)
            self.logger.info('Writing CPP file for service clients: ' + service_cpp_file_path)
            service_cpp_file = open(service_cpp_file_path,  'w+')

            service_cpp_file.write('/***********************************************************************\n')
            service_cpp_file.write('ROS service definition headers and ROS connector template instantiations.\n')
            service_cpp_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            service_cpp_file.write('Please do not modify it by hand.\n')
            service_cpp_file.write('The contents will be overwritten and re-generated.\n')
            service_cpp_file.write('************************************************************************/\n')
            service_cpp_file.write('\n\n')

            service_cpp_file.write('#include <' + self.__ros_service_client_generated_h_name + '>')
            service_cpp_file.write('\n\n')
            service_cpp_file.write(self.__ros_service_client_cpp_source_static + '\n\n')

            # Collect all ROS service definition namespaces - these are obviously needed to satisfy GCC when using
            # request and response definitions as template parameters!

            service_namespaces = []

            for service_type in sorted(header_files_per_service_type.keys()):
                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    # These are guaranteed to be of length 2
                    service_cpp_namespace = service_type.split('/')
                    if service_cpp_namespace[0] not in service_namespaces:
                        service_namespaces.append(service_cpp_namespace[0])

            for namespace in service_namespaces:
                service_cpp_file.write('using namespace ' + namespace + ';\n')

            service_cpp_file.write('\n')

            for service_type in sorted(header_files_per_service_type.keys()):
                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    # These are guaranteed to be of length 2
                    service_cpp_type = service_type.replace('/', '::')
                    service_cpp_request_type = service_cpp_type + 'Request'
                    service_cpp_response_type = service_cpp_type + 'Response'
                    service_cpp_file.write('template class ZyROSConnectorServiceClient<' + service_cpp_type + ', ' + service_cpp_request_type + ', ' + service_cpp_response_type + '>;\n')

            # Service client factory method
            service_cpp_file.write('boost::shared_ptr<ZyROSServiceClient> ZyROSConnectorServiceClientFactory::createServiceClient(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType)\n')
            service_cpp_file.write('{\n')

            service_cpp_file.write('\tbool supported = false;\n')
            service_cpp_file.write('\tboost::shared_ptr<ZyROSServiceClient> serviceClient;\n')

            for service_type in sorted(header_files_per_service_type.keys()):
                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    service_cpp_file.write('\t// Service client instance for ROS service type: ' + service_type + '\n')
                    service_cpp_type = service_type.replace('/', '::')

                    service_cpp_request_type = service_cpp_type + 'Request'
                    service_cpp_response_type = service_cpp_type + 'Response'

                    service_cpp_file.write('\tif (serviceType == "' + service_cpp_type + '")\n')
                    service_cpp_file.write('\t{\n')
                    service_cpp_file.write('\t\tsupported = true;\n')
                    service_cpp_file.write('\t\tserviceClient.reset(new ZyROSConnectorServiceClient<' + service_cpp_type + ', ' + service_cpp_request_type + ', ' + service_cpp_response_type + '>(rosNode, serviceURI, 10));\n')
                    service_cpp_file.write('\t}\n')

            service_cpp_file.write('\n\tif (supported)\n')
            service_cpp_file.write('\t{\n')
            service_cpp_file.write('\t\tmsg_info("ZyROSConnectorServiceClientFactory") << "ROS service type supported: " << serviceType;\n')
            service_cpp_file.write('\t}\n')
            service_cpp_file.write('\telse\n')
            service_cpp_file.write('\t{\n')
            service_cpp_file.write('\t\tmsg_warning("ZyROSConnectorServiceClientFactory") << "ROS service type NOT supported: " << serviceType;\n')
            service_cpp_file.write('\t}\n')

            service_cpp_file.write('\treturn serviceClient;\n')
            service_cpp_file.write('}\n')

            service_cpp_file.close()

            # ROS service server implementation template specializations
            service_cpp_file_path = os.path.join(self.__script_directory, '..', 'ZyROSConnector', self.__ros_service_server_generated_cpp_name)
            self.logger.info('Writing CPP file for service servers: ' + service_cpp_file_path)
            service_cpp_file = open(service_cpp_file_path,  'w+')

            service_cpp_file.write('/***********************************************************************\n')
            service_cpp_file.write('ROS service definition headers and ROS connector template instantiations.\n')
            service_cpp_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            service_cpp_file.write('Please do not modify it by hand.\n')
            service_cpp_file.write('The contents will be overwritten and re-generated.\n')
            service_cpp_file.write('************************************************************************/\n')
            service_cpp_file.write('\n\n')

            service_cpp_file.write(self.__ros_service_server_cpp_source_static + '\n')

            for service_type in sorted(header_files_per_service_type.keys()):
                self.logger.debug('Matching header files for service type \'' + service_type + '\': ' + str(len(header_files_per_service_type[service_type])))

                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    for header_file in header_files_per_service_type[service_type]:
                        self.logger.debug('WRITING INCLUDE STATEMENT FOR SERVICE: ' + '#include <' + header_file + '>')
                        service_cpp_file.write('#include <' + header_file + '>\n')

            service_cpp_file.write('// ROS service server template type instantiations\n')
            for service_type in sorted(header_files_per_service_type.keys()):
                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    # These are guaranteed to be of length 2
                    service_cpp_type = service_type.replace('/', '::')
                    service_cpp_request_type = service_cpp_type + 'Request'
                    service_cpp_response_type = service_cpp_type + 'Response'
                    service_cpp_request_handler = 'ZyROSConnectorServerRequestHandler<' + service_cpp_request_type + ', ' + service_cpp_response_type + '>'
                    service_cpp_file.write('template class ZyROSConnectorServiceServerImpl<' + service_cpp_request_type + ', ' + service_cpp_response_type + ', ' + service_cpp_request_handler + '>;\n')

            service_cpp_file.write('\n\n')

            service_cpp_file.write('// ROS service server worker thread template type instantiations\n')
            for service_type in sorted(header_files_per_service_type.keys()):
                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    # These are guaranteed to be of length 2
                    service_cpp_type = service_type.replace('/', '::')
                    service_cpp_request_type = service_cpp_type + 'Request'
                    service_cpp_response_type = service_cpp_type + 'Response'
                    service_cpp_request_handler = 'ZyROSConnectorServerRequestHandler<' + service_cpp_request_type + ', ' + service_cpp_response_type + '>'
                    service_cpp_file.write('template class ZyROSConnectorServiceServerWorkerThread<' + service_cpp_request_type + ', ' + service_cpp_response_type + ', ' + service_cpp_request_handler + '>;\n')

            service_cpp_file.close()


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    parser = argparse.ArgumentParser()
    parser.add_argument("base_directory", help="Base output directory for generated header files", type=str, default=os.getcwd())
    parser.add_argument("message_types", help="Space-separated list of ROS message types",
                        type=str, default="")
    parser.add_argument("service_types", help="Space-separated list of ROS service types",
                        type=str, default="")
    args = parser.parse_args()

    if args.message_types != '':
        logger_main.info('Starting auto-generation of ROS message bindings.')

        message_types_arg = args.message_types.replace('\n', ' ')
        message_types_list = message_types_arg.split(' ')
        logger_main.debug('Message types list: ' + str(repr(message_types_list)))
        message_binding_generator = ROSMessageBindingGenerator(base_directory=args.base_directory, ros_message_types=message_types_list)

        message_binding_generator.generate_binding_sources()
    else:
        logger_main.warning('No ROS message types have been specified, no source code files will be generated!')

    if args.service_types != '':
        logger_main.info('Starting auto-generation of ROS service bindings.')

        service_types_arg = args.service_types.replace('\n', ' ')
        service_types_list = service_types_arg.split(' ')
        logger_main.debug('Service types list: ' + str(repr(service_types_list)))
        service_binding_generator = ROSServiceBindingGenerator(base_directory=args.base_directory, ros_service_types=service_types_list)

        service_binding_generator.generate_binding_sources()
    else:
        logger_main.warning('No ROS service types have been specified, no source code files will be generated!')
