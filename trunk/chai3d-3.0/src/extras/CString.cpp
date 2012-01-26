//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \author    Dan Morris
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 387 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "extras/CString.h"
#include "math/CMaths.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
	Compute the length of a string up to 255 characters. If the end of string
    cannot be found, then -1 is returned as a result.

	\fn			int cStringLength(const char* a_input)
	\param		a_string  Input string. Pointer to a char.
	\return		Return the length of the string. If length cannot be computed,
                return -1.
*/
//===========================================================================
int cStringLength(const char* a_input)
{
	int counter = 0;
	while (counter < 256)
	{
		if (a_input[counter] == 0) { return (counter); }
		counter++;
	}
	return (-1);
}


//===========================================================================
/*!
    Convert a string into lower case.

    \fn     string cStringToLower(string& a_input)
    \param  a_string  Input string 
    \return  Returns the output string.
*/
//===========================================================================
string cStringToLower(string a_input)
{
    string result = a_input;
    unsigned int i;
    unsigned int length = (unsigned int)(result.length());
    for (i=0; i<length; i++)
    {
        result[i] = tolower(a_input[i]);
    }
    return (result);
}


//===========================================================================
/*!
    Finds the extension in a filename.

    \fn     string cFindFileExtension(string a_input, bool a_includeDot)
    \param  a_input  Input filename.
    \param  a_includeDot  If \b true, include the dot at the beginning of 
                          the extension. (example: ".jpg")
    \return  Returns a string containing the extension.
*/
//===========================================================================
string cFindFileExtension(string a_input, bool a_includeDot)
{
    string result = "";
    unsigned int length = (unsigned int)(a_input.length());
    unsigned int i = length;
    bool finished = false;

    while ((!finished) && (i > 0))
    {
        // move to next character
        i--;

        // check for extension "."
        if (a_input[i] == '.')
        {
            if (a_includeDot) {
                result = a_input.substr(i, length-i);
            } else {
                result = a_input.substr(i+1, length-(i+1));    
            }
            finished = true;
        }

        // check for '/' or '\\'
        else if ((a_input[i] == '/') || (a_input[i] == '\\'))
        {
            finished = true;
        }
    }

    return (result);
}


//===========================================================================
/*!
    Discards the path component of a filename and returns the filename itself,
    optionally including the extension.

    \fn     string cFindFilename(string a_string, bool a_includeFileExtension)
    \param  a_input   Input string containing path and filename
    \param  a_includeExtension Should the output include the extension?
    \return  Returns the output string.
*/
//===========================================================================
string cFindFilename(string a_input, bool a_includeFileExtension)
{
    string result = "";
    unsigned int length = (unsigned int)(a_input.length());
    unsigned int i = length;
    bool finished = false;

    while ((!finished) && (i > 0))
    {
        // move to next character
        i--;

        // check for '/' or '\\'
        if ((a_input[i] == '/') || (a_input[i] == '\\'))
        {
            result = a_input.substr(i+1, length-1);
            if (!a_includeFileExtension)
            {
                string extension = cFindFileExtension(result, true);
                if (extension.length() > 0)
                {
                    result = result.substr(0, result.length() - extension.length());
                }
            }
            finished = true;
        }
    }

    // no path has been found. the filename therfore corresponds to the 
    // entire string.
    if (!finished)
    {
        if (!a_includeFileExtension)
        {
            string extension = cFindFileExtension(a_input, true);
            if (extension.length() > 0)
            {
                result = a_input.substr(0, a_input.length() - extension.length());
            }
            else
            {
                result = a_input;
            }
        }
        else
        {
            result = a_input;
        }
    }

    // finally resturn result.
    return (result);
}


//===========================================================================
/*!
    Returns the string a_filename by replacing its extension with a new
    string provided by parameter a_extension.

    \fn     string cReplaceFileExtension(string a_filename, string a_extension)
    \param  a_filename The input filename
    \param  a_extension  The extension to replace a_input's extension with
    \return  Returns the output string.
*/
//===========================================================================
string cReplaceFileExtension(string a_filename, string a_extension)
{
    string extension = cFindFileExtension(a_filename, true);
    string result = a_filename.substr(0, a_filename.length()- extension.length());

    if (a_extension.length() > 0)
    {
        if (a_extension[0] == '.')
            result = result + a_extension;
        else
            result = result + "." + a_extension;
    }
    return (result);
}


//===========================================================================
/*!
    Finds only the _path_ portion of source, and copies it with
    _no_ trailing '\\'.  If there's no /'s or \\'s, writes an
    empty string

    \fn     bool find_directory(char* a_dest, const char* a_source)
    \param  a_dest    String which will contain the directory name
    \param  a_source  Input string containing path and filename
    \return true for success, false if there's no separator
*/
//===========================================================================
string cFindDirectory(string a_input)
{
    string filename = cFindFilename(a_input, true);
    string result = a_input.substr(0, a_input.length() - filename.length());
    return (result);
}


//===========================================================================
/*!
    Convert a \e boolean into a \e string.

    \fn       string cStr(const bool& a_value)
    \param    a_value  Input value of type \e boolean.
    \return   Return output string.
*/
//===========================================================================
string cStr(const bool& a_value)
{
    string result;
    if (a_value) 
        result = "true";
    else 
        result = "false";   
    return (result);
}


//===========================================================================
/*!
    Convert an \e integer into a \e string.

    \fn       string cStr(const int& a_value)
    \param    a_value  Input value of type \e integer.
    \return   Return output string.
*/
//===========================================================================
string cStr(const int& a_value)
{
    string result = "";
    char buffer[255];
    sprintf(buffer, "%d", a_value);
    result.append(buffer);
    return (result);
}


//===========================================================================
/*!
    Convert a \e float into a \e string.

    \fn       string cStr(const float& a_value, const unsigned int a_precision)
    \param    a_value  Input value of type \e float.
    \param    a_precision  Number of digits displayed after the decimal point.
    \return   Return output string.
*/
//===========================================================================
string cStr(const float& a_value, const unsigned int a_precision)
{
    string result = "";

    // make sure number of digits ranges between 0 and 20
    int numDigits = (int)a_precision;
    if (numDigits > 20)
    {
        numDigits = 20;
    }

	// if number of digits is zero, remove '.'
	if (numDigits == 0)
	{
		numDigits = -1;
	}

	char buffer[255];
	sprintf(buffer, "%.20f", a_value);
	buffer[ (cStringLength(buffer) - 20 + numDigits) ] = '\0';
	double chopped_value = atof(buffer);
	double round_diff = a_value - chopped_value;
	double round_threshold = 0.5f*pow(0.1f, (int)a_precision);
	if (fabs(round_diff) >= round_threshold)
	{
        double rounded_value;
		if (a_value >= 0.0) rounded_value = a_value + round_threshold;
		else rounded_value = a_value - round_threshold;

        sprintf(buffer, "%.20f", rounded_value);
	    buffer[ (cStringLength(buffer) - 20 + numDigits) ] = '\0';
	}
	result.append(buffer);

    return (result);
}


//===========================================================================
/*!
    Convert a \e double into a \e string.

    \fn       string cStr(const double& a_value, const unsigned int a_precision)
    \param    a_value  Input value of type \e double.
    \param    a_precision  Number of digits displayed after the decimal point.
    \return   Return output string.
*/
//===========================================================================
string cStr(const double& a_value, const unsigned int a_precision)
{
    string result = "";

    // make sure number of digits ranges between 0 and 20
    int numDigits = a_precision;
    if (numDigits > 20)
    {
      numDigits = 20;
    }

	// if number of digits is zero, remove '.'
	if (numDigits == 0)
	{
		numDigits = -1;
	}

	char buffer[255];
	sprintf(buffer, "%.20f", a_value);
	buffer[ (cStringLength(buffer) - 20 + numDigits) ] = '\0';
	double chopped_value = atof(buffer);
	double round_diff = a_value - chopped_value;
	double round_threshold = 0.5*pow(0.1, (int)a_precision);
	if (fabs(round_diff) >= round_threshold) 
	{
        double rounded_value;
		if (a_value >= 0.0) rounded_value = a_value + round_threshold;
		else rounded_value = a_value - round_threshold;

        sprintf(buffer, "%.20f", rounded_value);
	    buffer[ (cStringLength(buffer) - 20 + numDigits) ] = '\0';
	}
	result.append(buffer);

    return (result);
}
