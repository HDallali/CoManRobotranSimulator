/*
 * Functions to initialize the MBSdata structure from the <ProjectName>.mbsdata (xml format) file.
 *
 * Allan Barrea Feb. 2013
 */

//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//#include <libxml/xmlmemory.h>
//#include <libxml/parser.h>
//#include <libxml/tree.h>

//#include "user_sf_IO.h"
//#include "MBSfun.h"
//#include "sfdef.h"
//#include "MBSdef.h"

///*
// * loadMBSdata_xml --> opens and parse the .mbsdata (xml) file
// *                 --> calls loadMBSsizes_xml
// *                 --> calls loadMBSelements_xml
// *                           --> calls loadUserModel_xml
// */
//MBSdataStruct* loadMBSdata_xml(const char *filein)
//{
//    MBSdataStruct* MBSdata = NULL;

//    xmlDocPtr doc = NULL;
//	xmlNodePtr root = NULL;

//    /*
//     * This initialize the library and check potential ABI mismatches
//     * between the version it was compiled for and the actual shared
//     * library used.
//     */
//    LIBXML_TEST_VERSION

//    // Parse the file and get the DOM
//    doc = xmlReadFile(filein, NULL, 0);

//    if (doc == NULL ) {
//		fprintf(stderr, "error: could not parse file %s\n", filein);
//		return NULL;
//	}

//    // Get the root element node
//	root = xmlDocGetRootElement(doc);

//	if (root == NULL) {
//		fprintf(stderr, "empty document (file %s)\n", filein);
//		xmlFreeDoc(doc);
//		return NULL;
//	}

//	if (xmlStrcmp(root->name, (const xmlChar *) "mbsdata")) {
//		fprintf(stderr,"document of the wrong type, root node != 'mbsdata'\n");
//		xmlFreeDoc(doc);
//		return NULL;
//	}

//    // Print element names
//    //print_element_names_xml(root);

//    // Load MBSdata sizes
//    MBSdata = loadMBSsizes_xml(doc, root);

//    // Load MBSdata elements
//    loadMBSelements_xml(MBSdata, doc, root);

//    // Free the document
//    xmlFreeDoc(doc);

//    /*
//     * Free the global variables that may
//     * have been allocated by the parser.
//     */
//    xmlCleanupParser();

//    return MBSdata;
//}

#include "loadMBSdata_xml.h"

MBSdataStruct* loadMBSdata_xml(const char *mbs_xml_name)
{
    MDS_gen_strct *mds      = NULL;
    MBSdataStruct *MBSdata  = NULL;
    PART_gen_strct *part    = NULL;

    // retrieve data from xml .mbs
    mds = MDS_mbs_reader(mbs_xml_name);

    //load mbsdata struct
    MBSdata = MDS_create_MBSdataStruct(mds);

    // coordinate partitioning
    part = init_PART_gen_strct(mds);

    part->options->rowperm = 1;
    //part->options->visualise = 0;
    //part->options->treshold = 1e-9;
    //part->options->drivers = 0;
    //part->options->verbose = 0;
    //part->options->clearmbsglobal = 1;

    PART_run_part(mds, MBSdata, part);

    free_MDS_gen_strct(mds);
    free_PART_gen_strct(part);

    return MBSdata;
}
