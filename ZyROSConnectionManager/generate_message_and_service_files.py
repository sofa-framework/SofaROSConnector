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
        self.__ros_message_generated_cpp_name = 'ZyROS_MessageType_Instantiations.h'

    def generate_binding_sources(self):
        self.logger.info('Generating ROS message binding sources. Message definitions passed: ' + str(len(self.__ros_message_types)))

        header_files_per_message_type = self.find_matching_headers(self.__ros_message_types, True)
        if len(header_files_per_message_type.keys()) > 0:
            message_source_file_path = os.path.join(self.__base_directory, self.__ros_message_generated_cpp_name)
            self.logger.info('Writing header file for messages: ' + message_source_file_path)
            message_source_file = open(message_source_file_path, 'w+')

            message_source_file.write('/***********************************************************************\n')
            message_source_file.write('ROS message definition headers and ROS connector template instantiations.')
            message_source_file.write('This file is AUTO-GENERATED during the CMake run.')
            message_source_file.write('Please do not modify it by hand.')
            message_source_file.write('The contents will be overwritten and re-generated.')
            message_source_file.write('************************************************************************/\n')
            message_source_file.write('\n\n')

            message_source_file.write('#include <ZyROSConnector/ZyROSConnectorTopicSubscriber.h>')
            message_source_file.write('#include <ZyROSConnector/ZyROSConnectorTopicPublisher.h>')

            message_source_file.write('\n\n')

            for message_type in sorted(header_files_per_message_type.keys()):
                self.logger.debug('Matching header files for message type \'' + message_type + '\': ' + str(len(header_files_per_message_type[message_type])))

                message_source_file.write('// Publisher and subscriber proxy class instantiation for ROS message type: ' + message_type + '\n')
                for header_file in header_files_per_message_type[message_type]:
                    message_source_file.write('include <' + header_file + '>\n')
                    message_cpp_type = message_type.replace('/', '::')
                    message_source_file.write('template class ZyROSConnectorTopicSubscriber<' + message_cpp_type + '>;\n')
                    message_source_file.write('template class ZyROSConnectorTopicPublisher<' + message_cpp_type + '>;\n')

                message_source_file.write('\n')

            message_source_file.close()


class ROSServiceBindingGenerator(ROSBindingGenerator):

    def __init__(self, base_directory, ros_service_types = []):
        ROSBindingGenerator.__init__(self)
        self.__ros_service_types = ros_service_types
        self.__base_directory = base_directory
        self.__ros_service_generated_cpp_name = 'ZyROS_ServiceType_Instantiations.h'

    def generate_binding_sources(self):
        self.logger.info('Generating ROS service binding sources. Service definitions passed: ' + str(len(self.__ros_service_types)))

        header_files_per_service_type = self.find_matching_headers(self.__ros_service_types, False)
        if len(header_files_per_service_type.keys()) > 0:
            service_source_file_path = os.path.join(self.__base_directory, self.__ros_service_generated_cpp_name)
            self.logger.info('Writing header file for services: ' + service_source_file_path)
            service_source_file = open(service_source_file_path, 'w+')

            service_source_file.write('/***********************************************************************\n')
            service_source_file.write('ROS service definition headers and ROS connector template instantiations.')
            service_source_file.write('This file is AUTO-GENERATED during the CMake run.')
            service_source_file.write('Please do not modify it by hand.')
            service_source_file.write('The contents will be overwritten and re-generated.')
            service_source_file.write('************************************************************************/\n')
            service_source_file.write('\n\n')

            service_source_file.write('#include <ZyROSConnector/ZyROSConnectorServiceClient.h>')

            service_source_file.write('\n\n')

            for service_type in sorted(header_files_per_service_type.keys()):
                self.logger.debug('Matching header files for service type \'' + service_type + '\': ' + str(len(header_files_per_service_type[service_type])))

                service_source_file.write('// Publisher and subscriber proxy class instantiation for ROS service type: ' + service_type + '\n')
                for header_file in header_files_per_service_type[service_type]:
                    service_source_file.write('include <' + header_file + '>\n')

                service_cpp_type = service_type.replace('/', '::')
                service_source_file.write('template class ZyROSConnectorServiceClient<' + service_cpp_type + '>;\n')

                service_source_file.write('\n')

            service_source_file.close()


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
