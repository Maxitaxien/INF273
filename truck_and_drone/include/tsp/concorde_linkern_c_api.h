#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ConcordeLinkernHandle ConcordeLinkernHandle;

    ConcordeLinkernHandle *concorde_linkern_create(
        int n,
        const int *full_matrix, /* row-major n x n */
        int seed);

    void concorde_linkern_destroy(ConcordeLinkernHandle *handle);

    int concorde_linkern_improve(
        ConcordeLinkernHandle *handle,
        int *tour, /* length n, modified in place */
        int stallcount,
        int repeatcount,
        int kicktype,
        double time_bound,
        double *out_value);

#ifdef __cplusplus
}
#endif