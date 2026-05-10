/*
 * Système de détection de collision - Essaim de drones (UAV)
 * Approche : tri par axe X + balayage avec élagage
 * Complexité : O(n log n)
 * Contrainte : AUCUN crochet [] - uniquement arithmétique de pointeurs
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/* -------------------------------------------------------
 * Structure imposée par le cahier des charges
 * ------------------------------------------------------- */
struct Drone {
    int   id;
    float x;
    float y;
    float z;
};

/* -------------------------------------------------------
 * Calcul de la distance euclidienne 3D entre deux drones
 * ------------------------------------------------------- */
float distance(struct Drone *a, struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* -------------------------------------------------------
 * Comparateur pour qsort : trie les drones par axe X
 * ------------------------------------------------------- */
int comparer_x(const void *a, const void *b) {
    struct Drone *da = (struct Drone *)a;
    struct Drone *db = (struct Drone *)b;
    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}

/* -------------------------------------------------------
 * Programme principal
 * ------------------------------------------------------- */
int main(void) {

    int n = 10000;

    /* --- Allocation dynamique du bloc continu (le tas) --- */
    struct Drone *essaim = (struct Drone *)malloc(n * sizeof(struct Drone));
    if (essaim == NULL) {
        printf("Erreur : allocation impossible\n");
        return 1;
    }

    /* --- Initialisation des drones avec coordonnées aléatoires --- */
    /* Navigation par arithmétique de pointeurs : (essaim + i) */
    int i;
    for (i = 0; i < n; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 10000);
        (essaim + i)->y  = (float)(rand() % 10000);
        (essaim + i)->z  = (float)(rand() % 10000);
    }

    /* Injection d'une paire connue pour valider le résultat */
    (essaim + 0)->x = 500.0f; (essaim + 0)->y = 500.0f; (essaim + 0)->z = 500.0f;
    (essaim + 1)->x = 500.1f; (essaim + 1)->y = 500.0f; (essaim + 1)->z = 500.0f;

    /* --- Etape 1 : Tri de l'essaim par coordonnée X --- */
    /* qsort est O(n log n) — c'est le coût dominant de l'algorithme */
    qsort(essaim, n, sizeof(struct Drone), comparer_x);

    /* --- Etape 2 : Recherche de la paire la plus proche --- */
    /*
     * STRATÉGIE (balayage avec élagage) :
     * Après le tri, les drones proches sur X sont voisins dans le tableau.
     * Pour chaque drone i, on compare avec j = i+1, i+2, ...
     * mais on s'arrête dès que l'écart en X seul dépasse d_min :
     * car si |Xi - Xj| >= d_min, la distance 3D sera forcément >= d_min.
     * En pratique, très peu de comparaisons sont effectuées.
     */
    float d_min = FLT_MAX;
    int   id_a  = -1;
    int   id_b  = -1;

    int j;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {

            /* Elagage clé : écart X seul suffit à éliminer la paire */
            float ecart_x = (essaim + j)->x - (essaim + i)->x;
            if (ecart_x >= d_min) break;

            float d = distance(essaim + i, essaim + j);
            if (d < d_min) {
                d_min = d;
                id_a  = (essaim + i)->id;
                id_b  = (essaim + j)->id;
            }
        }
    }

    /* --- Affichage du résultat --- */
    printf("=== Résultat de l'analyse de collision ===\n");
    printf("Paire la plus proche : Drone %d et Drone %d\n", id_a, id_b);
    printf("Distance minimale    : %.4f m\n", d_min);

    if (d_min < 2.0f)
        printf("*** ALERTE : manoeuvre d'evitement requise ! ***\n");
    else
        printf("OK : distance de securite respectee.\n");

    /* --- Libération mémoire --- */
    free(essaim);
    return 0;
}