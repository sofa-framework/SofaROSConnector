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
        self.__ros_message_generated_h_name = 'ZyROS_MessageType_Instantiations.h'
        self.__ros_message_generated_cpp_name = 'ZyROS_MessageTypes_Instantiations.cpp'

        self.__excluded_message_names = [] # These break plugin loading when included directly under ROS kinetic

    def generate_binding_sources(self):
        self.logger.info('Generating ROS message binding sources. Message definitions passed: ' + str(len(self.__ros_message_types)))

        header_files_per_message_type = self.find_matching_headers(self.__ros_message_types, True)
        if len(header_files_per_message_type.keys()) > 0:
            message_source_file_path = os.path.join(self.__base_directory, self.__ros_message_generated_h_name)
            self.logger.info('Writing header file for messages: ' + message_source_file_path)
            message_source_file = open(message_source_file_path, 'w+')

            message_source_file.write('/***********************************************************************\n')
            message_source_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_source_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_source_file.write('Please do not modify it by hand.\n')
            message_source_file.write('The contents will be overwritten and re-generated.\n')
            message_source_file.write('************************************************************************/\n')
            message_source_file.write('\n\n')

            message_source_file.write('#include <ZyROSConnectorTopicSubscriber.h>\n')
            message_source_file.write('#include <ZyROSConnectorTopicPublisher.h>')

            message_source_file.write('\n\n')

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

            message_source_file.write('\t\tclass ZyROSConnectorMessageFactory\n')
            message_source_file.write('\t\t{\n')
            message_source_file.write('\t\tpublic:\n')
            message_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSListener> createTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType);\n')
            message_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSPublisher> createTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType);\n')
            message_source_file.write('\t\t};\n')

            message_source_file.write('\t}\n')
            message_source_file.write('}\n')

            message_source_file.close()

            message_cpp_file_path = os.path.join(self.__base_directory, self.__ros_message_generated_cpp_name)
            self.logger.info('Writing CPP file for messages: ' + message_cpp_file_path)
            message_cpp_file = open(message_cpp_file_path, 'w+')

            message_cpp_file.write('/***********************************************************************\n')
            message_cpp_file.write('ROS message definition headers and ROS connector template instantiations.\n')
            message_cpp_file.write('This file is AUTO-GENERATED during the CMake run.\n')
            message_cpp_file.write('Please do not modify it by hand.\n')
            message_cpp_file.write('The contents will be overwritten and re-generated.\n')
            message_cpp_file.write('************************************************************************/\n')
            message_cpp_file.write('\n\n')

            message_cpp_file.write('#include <' + self.__ros_message_generated_h_name + '>\n')
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
                        message_cpp_file.write('template class ZyROSConnectorTopicPublisher<' + message_cpp_type + '>;\n')

                        message_cpp_file.write('template <>\n')
                        message_cpp_file.write('void ZyROSConnectorTopicPublisher<' + message_cpp_type + '>::publishMessageQueue()\n')
                        message_cpp_file.write('{\n')
                        message_cpp_file.write('\tboost::mutex::scoped_lock lock(m_mutex);\n')
                        message_cpp_file.write('\tif (!m_messageQueue.empty())\n')
                        message_cpp_file.write('\t{\n')
                        message_cpp_file.write('\t\tmsg_info("ZyROSConnectorTopicPublisher") << "publishMessageQueue of size " << m_messageQueue.size();\n')
                        message_cpp_file.write('\t\twhile (!m_messageQueue.empty())\n')
                        message_cpp_file.write('\t\t{\n')
                        message_cpp_file.write('\t\t\t' + message_cpp_type + '& msg = m_messageQueue.back();\n')
                        message_cpp_file.write('\t\t\tm_publisher.publish(msg);\n')
                        message_cpp_file.write('\t\t\tm_messageQueue.pop_back();\n')
                        message_cpp_file.write('\t\t}\n')
                        message_cpp_file.write('\t}\n')
                        message_cpp_file.write('\tlock.unlock();\n')
                        message_cpp_file.write('}\n\n')

                        message_cpp_file.write('template <>\n')
                        message_cpp_file.write('void ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>::cleanup()\n')
                        message_cpp_file.write('{\n')
                        message_cpp_file.write('\tif (m_topicConnection.connected())\n')
                        message_cpp_file.write('\t\tm_topicConnection.disconnect();\n\n')
                        message_cpp_file.write('\tif (getMessageCount() > 0)\n')
                        message_cpp_file.write('\t{\n')
                        message_cpp_file.write('\t\tclearMessages();\n')
                        message_cpp_file.write('\t}\n')
                        message_cpp_file.write('}\n')

                message_cpp_file.write('\n')

            message_cpp_file.write('\n\n')

            message_cpp_file.write('boost::shared_ptr<ZyROSListener> ZyROSConnectorMessageFactory::createTopicSubscriber(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)\n')
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
                    message_cpp_file.write('\t\tconst boost::shared_ptr<ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>> tmp(new ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>(rosNode, topicURI, 50, true));\n')
                    message_cpp_file.write('\t\ttopicListener = boost::dynamic_pointer_cast<ZyROSListener>(tmp);\n')
                    message_cpp_file.write('\t}\n')


            message_cpp_file.write('\tif (supported)\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_info("ZyROSConnectorMessageFactory") << "ROS message type supported: " << messageType;\n')
            message_cpp_file.write('\t}\n')
            message_cpp_file.write('\telse\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_warning("ZyROSConnectorMessageFactory") << "ROS message type NOT supported: " << messageType;\n')
            message_cpp_file.write('\t}')

            message_cpp_file.write('\treturn topicListener;\n')

            message_cpp_file.write('}\n\n')

            message_cpp_file.write('boost::shared_ptr<ZyROSPublisher> ZyROSConnectorMessageFactory::createTopicPublisher(ros::NodeHandlePtr rosNode, const std::string& topicURI, const std::string& messageType)\n')
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
                    message_cpp_file.write('\t\tconst boost::shared_ptr<ZyROSConnectorTopicPublisher<' + message_cpp_type + '>> tmp(new ZyROSConnectorTopicPublisher<' + message_cpp_type + '>(rosNode, topicURI, 10));\n')
                    message_cpp_file.write('\t\ttopicPublisher = boost::dynamic_pointer_cast<ZyROSPublisher>(tmp);\n')
                    message_cpp_file.write('\t}\n')


            message_cpp_file.write('\tif (supported)\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_info("ZyROSConnectorMessageFactory") << "ROS message type supported: " << messageType;\n')
            message_cpp_file.write('\t}\n')
            message_cpp_file.write('\telse\n')
            message_cpp_file.write('\t{\n')
            message_cpp_file.write('\t\tmsg_warning("ZyROSConnectorMessageFactory") << "ROS message type NOT supported: " << messageType;\n')
            message_cpp_file.write('\t}\n')

            message_cpp_file.write('\treturn topicPublisher;\n')

            message_cpp_file.write('}\n')

            message_cpp_file.close()


class ROSServiceBindingGenerator(ROSBindingGenerator):

    def __init__(self, base_directory, ros_service_types = []):
        ROSBindingGenerator.__init__(self)
        self.__ros_service_types = ros_service_types
        self.__base_directory = base_directory
        self.__ros_service_generated_h_name = 'ZyROS_ServiceType_Instantiations.h'
        self.__ros_service_generated_cpp_name = 'ZyROS_ServiceTypes_Instantiations.cpp'

        self.__excluded_service_names = ['rosapi/GetParam'] # These break compilation when included directly under ROS kinetic

    def generate_binding_sources(self):
        self.logger.info('Generating ROS service binding sources. Service definitions passed: ' + str(len(self.__ros_service_types)))

        header_files_per_service_type = self.find_matching_headers(self.__ros_service_types, False)
        if len(header_files_per_service_type.keys()) > 0:
            service_source_file_path = os.path.join(self.__base_directory, self.__ros_service_generated_h_name)
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
            service_source_file.write('#include <ZyROSConnectorServiceServer.h>\n')

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

            for service_type in sorted(header_files_per_service_type.keys()):
                self.logger.debug('Matching header files for service type \'' + service_type + '\': ' + str(len(header_files_per_service_type[service_type])))

                service_is_excluded = False
                for excluded_service in self.__excluded_service_names:
                    if service_type.startswith(excluded_service):
                        self.logger.warning('Excluding ROS service definition: ' + service_type)
                        service_is_excluded = True

                if not service_is_excluded:
                    service_cpp_type = service_type.replace('/', '::')
                    service_cpp_request_type = service_cpp_type + 'Request'
                    service_cpp_response_type = service_cpp_type + 'Response'
                    service_source_file.write('\t\ttemplate class ZyROSConnectorServiceClient<' + service_cpp_type + ', ' + service_cpp_request_type + ', ' + service_cpp_response_type + '>;\n')

            service_source_file.write('\t\tclass ZyROSConnectorServiceFactory\n')
            service_source_file.write('\t\t{\n')
            service_source_file.write('\t\tpublic:\n')
            service_source_file.write('\t\t\tstatic boost::shared_ptr<ZyROSServiceClient> createServiceClient(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType);\n')
            service_source_file.write('// \t\t\tstatic boost::shared_ptr<ZyROSServiceServer> createServiceServer(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType);\n')
            service_source_file.write('\t\t};\n')

            service_source_file.write('\t}\n')
            service_source_file.write('}\n')

            service_source_file.close()

            service_cpp_file_path = os.path.join(self.__base_directory, self.__ros_service_generated_cpp_name)
            self.logger.info('Writing CPP file for services: ' + service_cpp_file_path)
            service_cpp_file = open(service_cpp_file_path, 'w+')

            service_cpp_file.write('/***********************************************************************\n')
            service_cpp_file.write('ROS service definition headers and ROS connector template instantiations.')
            service_cpp_file.write('This file is AUTO-GENERATED during the CMake run.')
            service_cpp_file.write('Please do not modify it by hand.')
            service_cpp_file.write('The contents will be overwritten and re-generated.')
            service_cpp_file.write('************************************************************************/\n')
            service_cpp_file.write('\n\n')

            service_cpp_file.write('#include <' + self.__ros_service_generated_h_name + '>')
            service_cpp_file.write('\n\n')
            service_cpp_file.write('using namespace Zyklio::ROSConnector;\n\n')

            service_cpp_file.write('boost::shared_ptr<ZyROSServiceClient> ZyROSConnectorServiceFactory::createServiceClient(ros::NodeHandlePtr rosNode, const std::string& serviceURI, const std::string& serviceType)\n')
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
                    service_cpp_file.write('\t\tconst boost::shared_ptr<ZyROSConnectorServiceClient<' + service_cpp_type + ', ' + service_cpp_request_type + ', ' + service_cpp_response_type + '>> tmp(new ZyROSConnectorServiceClient<' + service_cpp_type + ', ' + service_cpp_request_type + ', ' + service_cpp_response_type + '>(rosNode, serviceURI, 10));\n')
                    service_cpp_file.write('\t\tserviceClient = boost::dynamic_pointer_cast<ZyROSServiceClient>(tmp);\n')
                    service_cpp_file.write('\t}\n')

            service_cpp_file.write('\n\tif (supported)\n')
            service_cpp_file.write('\t{\n')
            service_cpp_file.write('\t\tmsg_info("ZyROSConnectorServiceFactory") << "ROS service type supported: " << serviceType;\n')
            service_cpp_file.write('\t}\n')
            service_cpp_file.write('\telse\n')
            service_cpp_file.write('\t{\n')
            service_cpp_file.write('\t\tmsg_warning("ZyROSConnectorServiceFactory") << "ROS service type NOT supported: " << serviceType;\n')
            service_cpp_file.write('\t}\n')

            service_cpp_file.write('\treturn serviceClient;\n')

            service_cpp_file.write('}\n')

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
