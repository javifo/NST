/*
 * gg_command_parser.c
 *
 *  Created on: Feb 3, 2011
 *      Author: x0151841
 */

#include <common.h>

#include "gg_command_parser.h"
#include "gg_def.h"
#include "gg_update.h"

static const char token_endline[] = { '\n', '\r', '\0' };
static const char token_tag_hex[] = { 'W', 'C', '\0' };
static const char token_tag_dec[] = { 'X', '\0' };
static const char token_tag_end[] = { ':', '\0' };
static const char token_separate[] = { ' ', '\0' };
static const char token_hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
		'9', 'A', 'B', 'C', 'D', 'E', 'F', '\0' };
static const char token_dec[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
		'9', '\0' };

static int script_is_token_of(const char token, const char *token_list)
{
	int res = -1;
	int i = 0;
	while (token_list[i]) {
		if (token_list[i] == token) {
			res = 0;
			break;
		}
		i++;
	}
	return res;
}

static int script_handle_tag_search(const char token, int *state,
		char *last_tag)
{
	if (!script_is_token_of(token, token_endline)) {
		return PARSER_OK;
	} else if (!script_is_token_of(token, token_tag_hex)
			|| !script_is_token_of(token, token_tag_dec)) {
		*last_tag = token;
		*state = STATE_TAG_END_SRH;
		return PARSER_OK;
	}
	return PARSER_UNKNOW_TAG;

}

static int script_handle_end_tag_search(const char token, int *state)
{
	if (!script_is_token_of(token, token_tag_end)) {
		*state = STATE_WITE_SPACE_SRH;
		return PARSER_OK;
	}
	return PARSER_BAD_TAG_SYNTAX;
}

static int script_handle_sperate_endl(const char token, int *state,
		const char last_tag, const char *line_buff,
		unsigned int *curr_line_size)
{
	int res;
	if (!script_is_token_of(token, token_separate)) {
		if (!script_is_token_of(last_tag, token_tag_dec))
			*state = STATE_DEC_SRH;
		else
			*state = STATE_HEX_BEG_SRH;
		return PARSER_OK;
	} else if (!script_is_token_of(token, token_endline)) {
		if (!script_is_token_of(last_tag, token_tag_dec)) {
			if (*curr_line_size < DEC_CMD_MIN_LEN)
				return PARSER_BAD_DATA;
		} else {
			if (*curr_line_size < HEX_CMD_MIN_LEN / 2)
				return PARSER_BAD_DATA;
		}
		res = gauge_single_command(last_tag,
				(unsigned char *) line_buff, *curr_line_size);
		if (res)
			return res;
		*curr_line_size = 0;
		*state = STATE_TAG_SRH;
		return PARSER_OK;
	}
	return PARSER_BAD_DATA_SYNTAX;

}

static int script_handle_hex_begin(const char token, int *state,
		char *line_buff, unsigned int *curr_line_size,
		unsigned int max_size, char *last_hex)
{
	if (!script_is_token_of(token, token_hex)) {
		*last_hex = token;
		*state = STATE_HEX_END_SRH;
		return PARSER_OK;
	}
	return PARSER_BAD_DATA_SYNTAX;

}

static int script_handle_hex_end(const char token, int *state, char *line_buff,
		unsigned int *curr_line_size, unsigned int max_size,
		const char last_hex)
{
	if (!script_is_token_of(token, token_hex)) {
		char tmp[2];
		char *tmp_end;
		if (*curr_line_size + 1 >= max_size)
			return PARSER_BUFFER_OVERFLOW;

		tmp[0] = last_hex;
		tmp[1] = token;
		tmp_end = tmp + 2;
		line_buff[*curr_line_size] = simple_strtoul(tmp, &tmp_end, 16);

		*curr_line_size += 1;
		*state = STATE_WITE_SPACE_SRH;
		return PARSER_OK;
	}
	return PARSER_BAD_DATA_SYNTAX;
}

static int script_handle_dec(const char token, int *state, char *line_buff,
		unsigned int *curr_line_size, unsigned int max_size,
		const char last_tag)
{
	int res;
	if (!script_is_token_of(token, token_dec)) {
		if (*curr_line_size + 1 >= max_size)
			return PARSER_BUFFER_OVERFLOW;
		line_buff[*curr_line_size] = token;
		*curr_line_size += 1;
		return PARSER_OK;
	} else if (!script_is_token_of(token, token_separate)) {
		return PARSER_OK;
	} else if (!script_is_token_of(token, token_endline)) {
		if (*curr_line_size < DEC_CMD_MIN_LEN)
			return PARSER_BAD_DATA;
		res = gauge_single_command(last_tag,
				(unsigned char *) line_buff, *curr_line_size);
		if (res)
			return res;
		*state = STATE_TAG_SRH;
		*curr_line_size = 0;
		return PARSER_OK;
	}
	return PARSER_BAD_DATA_SYNTAX;
}

static int script_handle_missing_end(int *state, char *line_buff,
		unsigned int *curr_line_size, unsigned int max_size,
		const char last_tag)
{
	int res;
	char endline_tag = token_endline[0];

	if (*state == STATE_WITE_SPACE_SRH || *state == STATE_DEC_SRH) {
		res = script_handle_sperate_endl(endline_tag, state, last_tag,
				line_buff, curr_line_size);
	} else {
		res = PARSER_UNEXPECTED_END;
	}
	return res;
}

int script_parser(const char *input_buffer, const unsigned int size)
{
	int state, res, i;
	char last_tag, last_hex;
	char line_buff[MAX_LINE_LEN];
	unsigned int curr_line_size;

	res = PARSER_GENERAL_ERROR;
	curr_line_size = 0;
	state = STATE_TAG_SRH;

	last_tag = 0;
	last_hex = 0;

	for (i = 0; i < size; ++i) {
		switch (state) {
		case STATE_TAG_SRH:
			res = script_handle_tag_search(input_buffer[i], &state,
					&last_tag);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		case STATE_TAG_END_SRH:
			res = script_handle_end_tag_search(input_buffer[i],
					&state);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		case STATE_WITE_SPACE_SRH:
			res = script_handle_sperate_endl(input_buffer[i],
					&state, last_tag, line_buff,
					&curr_line_size);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		case STATE_HEX_BEG_SRH:
			res = script_handle_hex_begin(input_buffer[i], &state,
					line_buff, &curr_line_size,
					sizeof(line_buff), &last_hex);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		case STATE_HEX_END_SRH:
			res = script_handle_hex_end(input_buffer[i], &state,
					line_buff, &curr_line_size,
					sizeof(line_buff), last_hex);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		case STATE_DEC_SRH:
			res = script_handle_dec(input_buffer[i], &state,
					line_buff, &curr_line_size,
					sizeof(line_buff), last_tag);
			CHECK_ERROR_EXIT(res != PARSER_OK);
			break;
		default:
			res = PARSER_UNEXPECTED_STATE;
			CHECK_ERROR_EXIT(res != PARSER_OK);

		}
	}

	if (curr_line_size > 0) {
		res = script_handle_missing_end(&state, line_buff,
				&curr_line_size, sizeof(line_buff), last_tag);
		CHECK_ERROR_EXIT(res != PARSER_OK);
	}
	if (state == STATE_TAG_SRH && curr_line_size == 0)
		return PARSER_OK;

	EXIT_WITH_ERROR;

	return res;
}

