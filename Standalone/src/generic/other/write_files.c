/*
 * Write files functions
 *
 * authors: Nicolas Van der Noot and Allan Barrea
 */

#include "main_simulation.h"
#include "useful_functions.h"

#ifdef WRITE_FILES

/*
 * Initialize a 'Write_files' structure
 */
Write_files* init_write_files(int nstep, int njoint)
{
    Write_files* write_files;

    write_files = (Write_files*) malloc(sizeof(Write_files));

    write_files->t = get_double_vec(nstep);

    write_files->qq = get_double_tab(njoint, nstep);

    write_files->out_vec = get_double_tab(NB_OUTPUT_VEC, nstep);

    write_files->kount = 0;

    return write_files;
} 

/*
 * Free a 'Write_files' structure
 */
void free_write_files(Write_files *write_files, int njoint)
{
	#ifndef WIN32 // strange bug with Windows -> to investigate
		free_double_vec(write_files->t);
		free_double_tab(write_files->qq, njoint);
		free_double_tab(write_files->out_vec, NB_OUTPUT_VEC);
	#endif

    free(write_files);
}


/*
 * Writes the .anim file.
 * Returns 0 if no problem and 1 if an error occured.
 */
int write_anim_file(Write_files* write_files, int njoint, const char *fileout)
{
    int kount;
    double *t;
    double**qq;
    
	int i = 0;
	int j = 0;
	FILE* fid = NULL; // internal filename

    t     = write_files->t;
    qq    = write_files->qq;
    kount = write_files->kount; // - 1; 

    // Opening file
    fid = fopen(fileout, "w"); // external filename
    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", fileout);
        return 1;
    }

    // Dumping values
    for (j=1; j<=kount; j++)
    {
        fprintf(fid, "%12.8f ", t[j]);
        
        for (i=0; i<njoint; i++)
        {
            fprintf(fid, "%12.8f ", qq[i][j]);
        }
        
        fprintf(fid, "\n");        
    }

    // Closing file
    fclose(fid);

    return 0;
}

int write_out_files(Write_files* write_files, const char generic_fileout[PATH_MAX_LENGTH])
{
    int kount;
    double **out_vec;
    
    int i;
    int j;

    FILE* fid = NULL; // internal filename
    char cur_fileout[PATH_MAX_LENGTH];

    out_vec = write_files->out_vec;
    kount   = write_files->kount;

    for (i=0; i<NB_OUTPUT_VEC; i++)
    {
        sprintf (cur_fileout, "%s_%d.txt", generic_fileout, i+1);
        fid = fopen(cur_fileout, "w"); // external filename

        if(fid == NULL)
        {
            printf("error: cannot open file '%s'\n", cur_fileout);
            return 1;
        }

        // Dumping values
        for (j=1; j<=kount; j++)
        {
            fprintf(fid, "%12.8f\n", out_vec[i][j]);     
        }

        // Closing file
        fclose(fid);
    }

    return 0;
}

#endif

/*
 * Print values of MBSdata (used for debug)
 */
void printMbsData(MBSdataStruct *MBSdata)
{
    int i = 0;
    printf("q = [");
    for(i = 1;  i <= MBSdata->njoint; i++){
        printf("%f", MBSdata->q[i]);
    }
    printf("] \n");
}