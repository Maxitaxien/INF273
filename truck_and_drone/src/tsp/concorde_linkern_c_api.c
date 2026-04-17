#include "tsp/concorde_linkern_c_api.h"

#include <stdlib.h>
#include <string.h>

#define CC_NETREADY

#include "util.h"
#include "linkern.h"

struct ConcordeLinkernHandle
{
    int n;
    CCdatagroup dat;
    CCrandstate rstate;
    int *good_edges; /* [u0,v0,u1,v1,...] */
    int good_ecount; /* number of undirected edges */
    int *outcycle;   /* length n */
};

static int build_dat_from_matrix(ConcordeLinkernHandle *h, const int *full_matrix)
{
    int n = h->n;
    int ecount = n * (n - 1) / 2;

    int *elist = (int *)malloc((size_t)(2 * ecount) * sizeof(int));
    int *elen = (int *)malloc((size_t)ecount * sizeof(int));
    if (!elist || !elen)
    {
        free(elist);
        free(elen);
        return 1;
    }

    int k = 0;
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            elist[2 * k] = i;
            elist[2 * k + 1] = j;
            elen[k] = full_matrix[i * n + j];
            ++k;
        }
    }

    int rval = CCutil_graph2dat_matrix(n, ecount, elist, elen, 0, &h->dat);

    free(elist);
    free(elen);
    return rval;
}

static int build_complete_good_edges(ConcordeLinkernHandle *h)
{
    int n = h->n;
    h->good_ecount = n * (n - 1) / 2;
    h->good_edges = (int *)malloc((size_t)(2 * h->good_ecount) * sizeof(int));
    if (!h->good_edges)
    {
        return 1;
    }

    int k = 0;
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            h->good_edges[2 * k] = i;
            h->good_edges[2 * k + 1] = j;
            ++k;
        }
    }
    return 0;
}

ConcordeLinkernHandle *concorde_linkern_create(int n, const int *full_matrix, int seed)
{
    if (n <= 0 || !full_matrix)
    {
        return NULL;
    }

    ConcordeLinkernHandle *h = (ConcordeLinkernHandle *)calloc(1, sizeof(ConcordeLinkernHandle));
    if (!h)
    {
        return NULL;
    }

    h->n = n;
    CCutil_init_datagroup(&h->dat);
    CCutil_sprand(seed, &h->rstate);

    if (build_dat_from_matrix(h, full_matrix))
    {
        concorde_linkern_destroy(h);
        return NULL;
    }

    if (build_complete_good_edges(h))
    {
        concorde_linkern_destroy(h);
        return NULL;
    }

    h->outcycle = (int *)malloc((size_t)n * sizeof(int));
    if (!h->outcycle)
    {
        concorde_linkern_destroy(h);
        return NULL;
    }

    return h;
}

void concorde_linkern_destroy(ConcordeLinkernHandle *h)
{
    if (!h)
        return;
    CCutil_freedatagroup(&h->dat);
    free(h->good_edges);
    free(h->outcycle);
    free(h);
}

int concorde_linkern_improve(
    ConcordeLinkernHandle *h,
    int *tour,
    int stallcount,
    int repeatcount,
    int kicktype,
    double time_bound,
    double *out_value)
{
    if (!h || !tour || !out_value)
    {
        return 1;
    }

    double val = 0.0;

    int rval = CClinkern_tour(
        h->n,
        &h->dat,
        h->good_ecount,
        h->good_edges,
        stallcount,
        repeatcount,
        tour,
        h->outcycle,
        &val,
        1,
        time_bound,
        -1.0,
        (char *)0,
        kicktype,
        &h->rstate);

    if (rval != 0)
    {
        return rval;
    }

    memcpy(tour, h->outcycle, (size_t)h->n * sizeof(int));
    *out_value = val;
    return 0;
}
