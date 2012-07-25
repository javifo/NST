/*
 * gg_command_parser.h
 *
 *  Created on: Feb 3, 2011
 *      Author: x0151841
 */

#ifndef GG_COMMAND_PARSER_H_
#define GG_COMMAND_PARSER_H_

#define HEX_CMD_MIN_LEN			6
#define DEC_CMD_MIN_LEN			1

#define STATE_TAG_SRH			0
#define STATE_TAG_END_SRH		1
#define STATE_WITE_SPACE_SRH		2
#define STATE_HEX_BEG_SRH		3
#define STATE_HEX_END_SRH		4
#define STATE_DEC_SRH			5

#define MAX_LINE_LEN			1024

#define PARSER_OK			0
#define PARSER_GENERAL_ERROR		-1
#define PARSER_UNKNOW_TAG		-2
#define PARSER_BAD_TAG_SYNTAX		-3
#define PARSER_BAD_DATA_SYNTAX		-4
#define PARSER_UNEXPECTED_STATE		-5
#define PARSER_BUFFER_OVERFLOW		-6
#define PARSER_BAD_DATA			-7
#define PARSER_UNEXPECTED_END		-8

int script_parser(const char *input_buffer, const unsigned int size);

#endif /* GG_COMMAND_PARSER_H_ */

