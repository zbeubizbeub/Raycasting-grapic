

#include<Grapic.h>
#include<math.h>

using namespace grapic;
using namespace std;

//////////////////////////////§CONSTANTE§///////////////////////////////////////////////////////////////

const int mapX=10, mapY=20;
const int dimw = 750;
const int dimh = 1520;
const int l = 10;
const float speed = 0.03;
const int nbr_faisceau = 128;
const float daor = M_PI * ((float) 128/ (nbr_faisceau)) /180   ;
const int maxmonstre = 45;
const int f = 0;
const int NBRMAXNODE = 150;
const float speedMonster = 1;
///////////////////////////////////////////////////////////////////////////////////////////////////////

int mapa[mapX][mapY] =
{
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},

    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},

    {1,0,0,1,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,1},

    {1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},

    {1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1},

    {1,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,1},

    {1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1},

    {1,0,0,0,0,0,1,1,1,1,1,1,1,0,0,1,0,0,0,1},

    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},

    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
};


struct Complex{
    float x;
    float y;
};

Complex make_Complex(float a, float b){
    struct Complex z;
    z.x = a;
    z.y = b;
    return z;
}

Complex make_Complex_expo(float r, float theta_deg){
    Complex z;
    z = make_Complex(r*cos(theta_deg*M_PI / 180), r*sin(theta_deg*M_PI / 180));
    return z;
}

Complex operator+(Complex a, Complex b){
    Complex res;
    res.x= a.x + b.x;
    res.y= a.y + b.y;
    return res;
}

Complex operator-(Complex a, Complex b){
    Complex res;
    res.x= a.x - b.x;
    res.y= a.y - b.y;
    return res;
}

Complex operator*(float lamda, Complex b){
    Complex res;
    res.x= lamda * b.x;
    res.y= lamda * b.y;
    return res;
}

Complex operator*(Complex a, Complex b){
    Complex res;
    res.x = a.x * b.x - a.y * b.y;
    res.y = a.x * b.y + a.y * b.x;
    return res;
}

Complex Rotate(Complex p, float cx, float cy, float theta){
    Complex D= make_Complex(cx, cy);
    Complex rot = make_Complex_expo(1, theta);
    return (p-D) * rot + D;
}

Complex coefinter(Complex A, Complex B, float r){
    Complex C;
    C.x = A.x * (1-r) + B.x *r;
    C.y = A.y * (1-r) + B.y *r;
    return C;
}
#define Vec2 Complex

////////////////////////////§STRUCTURE§//////////////////////////////////////////

// Structure contenant les images du jeu
struct Data{
    Image im;            // Image du niveau
    Image mur;           // Image des murs
    Image murbas;        // Image des murs du bas
    Image playergun;     // Image du joueur avec son arme
    Image playergunfire; // Image du joueur en train de tirer
    Image minimap;       // Image de la mini-map
    Image sol;           // Image du sol
    Image mort;          // Image de la mort du joueur
};

// Structure représentant un monstre
struct monster
{
    Vec2 monsterpos;     // Position du monstre
};

// Structure contenant un tableau de monstres
struct bestiaire
{
    int nbmonstre;                   // Nombre de monstres
    monster monstertab[maxmonstre];  // Tableau de monstres
};

// Structure représentant le joueur
struct player
{
    Vec2 playerpos, pd;  // Position et Vecteur direction du joueur
    float aor = 1; // angle de rotation en radian
};

// Structure représentant un vecteur avec sa longueur
struct Vecteur
{
    Vec2 pos;            // Position du vecteur
    float longueur;      // Longueur du vecteur
    float longueur2;     // Longueur au carré du vecteur
};

// Structure représentant un noeud dans un graphe de recherche
struct Node
{
    int f;               // f = g + h
    int g;               // coût de déplacement
    int h;               // distance à la cible
    Complex fils;        // noeud précédent
};

// Structure contenant un groupe de noeuds
struct GroupNode
{
    int nbrNode = 0;              // Nombre de noeuds dans le groupe
    Node tabNode[NBRMAXNODE];     // Tableau de noeuds
};
////////////////////////§IMAGE§//////////////////////////////////////////////

// Initialise les images utilisées dans le programme
void Initimage(Data& d)
{
    d.im = image("data/RAYCASTING/STBAR.png"); // Image d'arrière-plan
    d.mur = image("data/RAYCASTING/mur2haut.jpg"); // Image de mur en hauteur
    d.murbas = image("data/RAYCASTING/mur2.jpg"); // Image de mur en bas
    d.playergun = image("data/RAYCASTING/gun.png"); // Image de l'arme
    d.playergunfire = image("data/RAYCASTING/gunfire.png"); // Image de l'arme lorsqu'elle tire
    d.minimap = image("data/RAYCASTING/image.png"); // Image de la carte
    d.mort = image("data/RAYCASTING/mort.png"); // Image de la mort du joueur
}

// Dessine l'image de fond et l'image de l'arme du joueur
void drawimage(Data d)
{
    // Dessine l'image de fond en (0,0) avec une largeur de dimh et une hauteur de dimw/8
    image_draw(d.im, 0, 0, dimh, dimw/8);

    // Si la touche 'm' est pressée, dessine l'image de l'arme en train de tirer, sinon dessine l'image de l'arme normale
    if(isKeyPressed('m'))
        image_draw(d.playergunfire, 640, dimw / 8 , 250, 150);
    else
        image_draw(d.playergun, 640, dimw / 8 , 250, 150);
}

////////////////////////§PLAYER§//////////////////////////////////////////////

// Initialise la position de départ du joueur
void initplayer(player& p)
{
    p.playerpos = make_Complex( l *2 + l/2 , l * 2 + l/2); // Position initiale du joueur
}

// Dessine le joueur à l'écran
void drawplayer (player p)
{
    color(255, 205, 0); // Couleur du joueur
    circleFill(p.playerpos.x, p.playerpos.y, 2); // Dessine un cercle rempli à la position du joueur
    line( p.playerpos.x , p.playerpos.y , p.playerpos.x + p.pd.x *20 , p.playerpos.y  + p.pd.y *20); // Dessine une ligne indiquant la direction du joueur
}

void move_player(player& p){

    // On calcule la case dans laquelle se trouve le joueur
    Complex coPlayer = make_Complex(floor (p.playerpos.x/10), floor(p.playerpos.y/10));

    // On active le mode de répétition de touche pour permettre la saisie continue
    setKeyRepeatMode(true);

    // Si la touche 'q' est enfoncée, on tourne le joueur vers la gauche
    if (isKeyPressed('q'))
    {
        p.aor+=speed; // On augmente l'angle de rotation du joueur
        if (p.aor<0) { // Si l'angle devient négatif, on ajoute 2pi pour le ramener dans l'intervalle [0, 2pi[
            p.aor+=2*M_PI;
        }
        p.pd.x = -cos(p.aor)/2;// On calcule la composante en x du vecteur de direction du joueur
        p.pd.y=-sin(p.aor)/2;// On calcule la composante en y du vecteur de direction du joueur
    }

    // Si la touche 'd' est enfoncée, on tourne le joueur vers la droite
    if (isKeyPressed('d'))
    {
        p.aor-=speed; // On diminue l'angle de rotation du joueur
        if (p.aor>2*M_PI) { // Si l'angle dépasse 2pi, on le ramène dans l'intervalle [0, 2pi[
            p.aor-=2*M_PI;
        }
        p.pd.x = -cos(p.aor)/2; // On calcule la composante en x du vecteur de direction du joueur
        p.pd.y=-sin(p.aor)/2; // On calcule la composante en y du vecteur de direction du joueur
    }

    // Si la touche 'z' est enfoncée, on déplace le joueur vers l'avant
    if (isKeyPressed('z'))
    {
        // Si la touche 'j' est également enfoncée, on accélère le mouvement du joueur
        if (isKeyPressed('j')){
            p.playerpos.x+=p.pd.x*2; // On déplace le joueur de deux fois sa composante en x du vecteur de direction
            p.playerpos.y+=p.pd.y*2; // On déplace le joueur de deux fois sa composante en y du vecteur de direction
        }
        else {
            p.playerpos.x+=p.pd.x/2; // On déplace le joueur de moitié sa composante en x du vecteur de direction
            p.playerpos.y+=p.pd.y/2; // On déplace le joueur de moitié sa composante en y du vecteur de direction
        }
    }

    // Si la touche 's' est enfoncée, on recule le joueur
    if (isKeyPressed('s'))
    {
        p.playerpos.x-=p.pd.x/2; // On déplace le joueur de moitié sa composante en x du vecteur de direction en sens opposé
        p.playerpos.y-=p.pd.y/2; // On déplace le joueur de moitié sa composante en y du vecteur de direction en sens opposé
    }
}
///////////////////////§COLLISION§//////////////////////////////

// Cette fonction prend en entrée une référence à un objet player et gère les collisions entre le joueur et les murs du labyrinthe
void player_collision(player& d)
{
    // On calcule les positions précédentes du joueur en x et y
    float playerposbeforex = d.playerpos.x/10, playerposbeforey = d.playerpos.y/10;


    // On calcule les indices de la case dans laquelle se trouve le joueur actuellement
    int caseX = floor((d.playerpos.x )/10), caseY = floor((d.playerpos.y )/10);


    // On calcule les coefficients de déplacement en x et y entre la case précédente et la case actuelle
    float coefx = - caseX + playerposbeforex, coefy = - caseY + playerposbeforey ;


    // Si la case actuelle est un mur, on gère la collision
    if (mapa[caseX][caseY]==1) {
        if(coefx < 0.2){
            // Si le coefficient de déplacement en x est inférieur à 0.2, on recule le joueur d'un pixel en x
            d.playerpos.x = -1 + d.playerpos.x;
        }
        if(coefx > 0.2){
            // Si le coefficient de déplacement en x est supérieur à 0.2, on avance le joueur d'un pixel en x
            d.playerpos.x = 1 + d.playerpos.x;
        }
        if(coefy < 0.2){
            // Si le coefficient de déplacement en y est inférieur à 0.2, on recule le joueur d'un pixel en y
            d.playerpos.y = -1 + d.playerpos.y;
        }
        if(coefy > 0.2){
            // Si le coefficient de déplacement en y est supérieur à 0.2, on avance le joueur d'un pixel en y
            d.playerpos.y = 1 + d.playerpos.y;
        }
    }
 }

// Fonction booléen qui vérifie si le joueur est mort en regardant si la case sur laquelle il se trouve correspond à un mur de feu (code 4 dans la carte)
bool PlayerDead(player p)
{
    // On calcule la case de la carte correspondant à la position actuelle du joueur
    int caseX = floor((p.playerpos.x )/10), caseY = floor((p.playerpos.y )/10);


    // Si la case de la carte est un mur de feu, le joueur est mort, on affiche un message et on retourne true
    if(mapa[caseX][caseY]==4)
    {
        return true;
    }
    // Sinon, le joueur est vivant, on retourne false
    else return false;
}

///////////////////////§RAYCASTER§//////////////////////////////////////////////

int draw_rayon(player p,Vecteur& v, monster m,  int nbI)
{
    // Initialisation des variables
    Vec2 Cd = p.playerpos, Cs;  // Cd : position courante du rayon, Cs : position intermédiaire du rayon
    int monsterposx, monsterposy, caseX, caseY; // Variables pour stocker la position du monstre et la case dans laquelle se trouve le joueur
    float playerposbeforex, playerposbeforey, r; // Variables pour stocker la position précédente du joueur et la variable d'interpolation

    // Boucle de test du rayon
    for(int i = 0; i<nbI; i++)
    {
        r = (float) (i+1.0)/nbI; // Calcul de la variable d'interpolation
        playerposbeforex = Cd.x/10; // Calcul de la position précédente du joueur sur l'axe des abscisses
        playerposbeforey = Cd.y/10; // Calcul de la position précédente du joueur sur l'axe des ordonnées
        caseX = floor(playerposbeforex); // Récupération de la case dans laquelle se trouve le joueur sur l'axe des abscisses
        caseY = floor(playerposbeforey); // Récupération de la case dans laquelle se trouve le joueur sur l'axe des ordonnées

        Cs = coefinter(p.playerpos, p.playerpos  + 10* v.pos , r); // Calcul de la position intermédiaire du rayon

        // Test de collision avec les murs
        if(mapa[caseX][caseY] != 1 && mapa[caseX][caseY] != 4)
        {
            Cd = Cs; // Si pas de collision, on met à jour la position courante du rayon
        }
    }

    color(255, 0, 0); // Couleur du rayon
    line(p.playerpos.x,p.playerpos.y,Cd.x,Cd.y); // Dessin du rayon
    v.longueur = sqrt(pow((Cd.x) - p.playerpos.x ,  2 ) + pow(Cd.y - p.playerpos.y , 2)); // Calcul de la longueur du vecteur de direction du joueur
}

void draw_rayon_tir(player& p,Vecteur& v,  int nbI)
{
    // On initialise la position de départ du rayon à la position du joueur
    Vec2 Cd = p.playerpos, Cs;

    // On initialise la direction du rayon à la direction de tir du joueur


    float r;

    // Si la touche 'm' est appuyée
    if(isKeyPressed('m')){

        // Pour chaque segment qui compose le faisceau
        for(int i = 0; i<nbI; i++){

            // On calcule le ratio de la longueur du segment par rapport à la longueur totale du faisceau
            r = (float) (i+1.0)/nbI;

            // On calcule la position du joueur avant de se déplacer le long du segment
            float playerposbeforex = Cd.x/10;
            float playerposbeforey = Cd.y/10;

            // On récupère la case dans laquelle se trouve le joueur avant de se déplacer le long du segment
            int caseX = floor(playerposbeforex);
            int caseY = floor(playerposbeforey);

            // On calcule la position du point final du segment en fonction de sa longueur et de sa direction
            Cs = coefinter(p.playerpos, p.playerpos  + 100* p.pd , r);

            // Si la case dans laquelle on se trouve n'est pas un mur ni un monstre, on peut continuer à avancer
            if(mapa[caseX][caseY] != 1 && mapa[caseX][caseY] != 4){
                Cd = Cs;
            }

            // Si la case dans laquelle on se trouve est un monstre, on le tue
            if(mapa[caseX][caseY] == 4){
                mapa[caseX][caseY] = 0;
            }
        }

        // On dessine le faisceau
        color(255, 0, 0);
        line(p.playerpos.x,p.playerpos.y,Cd.x,Cd.y);
    }
}

///////////////////////§mur!§//////////////////////////////

void wall_draw(player& d, Vecteur& v, Data& a, monster& m, float nBi)
{
    for(int i = 0; i< nbr_faisceau/2; i++)
    {
            v.pos.x = -cos(d.aor + daor/2 * i)*20; // on d�finit pos.x avec cos, l'angle de rotation aor et i
            v.pos.y =-sin(d.aor + daor/2 * i)*20;  // on d�finit pos.y avec sin, l'angle de rotation aor et i
            draw_rayon(d, v, m, nBi);

            float dx = (dimh/2)/55; //on definit l'�cart entre chaque rectangle de mur
            float wallheight; // coef pour g�rer la taille des murs


            wallheight = (dimw/2.0) /v.longueur;
            image_draw(a.mur, dimh/2 - i * dx, dimw/2 -10, 15, 5 * wallheight);
            image_draw(a.murbas, dimh/2 - i * dx, (dimw/ 2 - 5 * wallheight), 15, dimw/2 - (dimw/ 2 - 5 * wallheight));
    }

    for(int i =0; i< nbr_faisceau/2; i++)
    {
            v.pos.x = -cos(d.aor - daor/2 * i)*20;
            v.pos.y =-sin(d.aor - daor/2 * i)*20;
            draw_rayon(d, v,m,nBi);

            float dx = (dimh/2)/55; //on definit l'�cart entre chaque rectangle de mur
            float wallheight; // coef pour g�rer la taille des murs


            wallheight = (dimw/2.0) / v.longueur;
            image_draw(a.mur, dimh/2 + i * dx, dimw/2 -10 , 15, 5 * wallheight);
            image_draw(a.murbas, dimh/2 + i * dx, (dimw/ 2 - 5 * wallheight), 15, dimw/2 - (dimw/ 2 - 5 * wallheight));
    }
}

/////////////////////////////§!monster!§////////////////////////////////////////

// Cette fonction initialise la position initiale du monstre.
// Elle prend en paramètre une structure monstre m qui contient une position de monstre sous forme de complexe.
void initmonster(monster &m){
   m.monsterpos = make_Complex (l * 5 + l/2, l* 10 + l/2) ;
}

// Cette fonction dessine le monstre sur l'écran.
// Elle prend en paramètre une structure monstre m qui contient une position de monstre sous forme de complexe.
void drawmonster(monster &m){
    color(0, 255 , 0); // On définit la couleur en RGB du cercle qui représente le monstre.
    circleFill(m.monsterpos.x, m.monsterpos.y, 5); // On dessine le cercle qui représente le monstre.
}

// Cette fonction met à jour la carte que le monstre voit.
// Elle prend en paramètre une structure monstre m qui contient une position de monstre sous forme de complexe.
void monster_view(monster &m){

    float monsterposbeforex = m.monsterpos.x/10; // On calcule la position X du monstre sur la carte.
    float monsterposbeforey = m.monsterpos.y/10; // On calcule la position Y du monstre sur la carte.

    int caseX = floor(monsterposbeforex); // On calcule la position de la case X sur la carte en arrondissant à l'entier inférieur.
    int caseY = floor(monsterposbeforey); // On calcule la position de la case Y sur la carte en arrondissant à l'entier inférieur.

    if(mapa[caseX][caseY]==1)mapa[caseX][caseY]=1; // Si la case est un mur, on la marque en tant que mur sur la carte du monstre.
    else mapa[caseX][caseY]=4; // Sinon, on la marque en tant que case monstre sur la carte.

    // Les lignes suivantes marquent les cases adjacentes en tant que sol sur la carte. Commentées car non-utilisées.
    //if(mapa[caseX -1][caseY]==4)mapa[caseX -1][caseY]=0;
    //if(mapa[caseX ][caseY-1]==4)mapa[caseX ][caseY-1]=0 ;
    //if(mapa[caseX +1][caseY]==4)mapa[caseX +1][caseY]=0 ;
    //if(mapa[caseX ][caseY+1]==4)mapa[caseX][caseY+1]=0 ;
}

// Cette fonction met à jour la position du monstre sur la carte.
// Elle prend en paramètre une structure monstre m qui contient une position de monstre sous forme de complexe.

void updatemonster(monster& m){
    monster_view(m); // On met à jour la procédure monster-view().
}

////////////////////////§GROUP_OF_MONSTER§////////////////////////////////////

void initgroupemonster (bestiaire &b, monster& m, int nbm){
    for(int i = 0; i < nbm; i++){
        initmonster(b.monstertab[i]);
    }
}

void groupdrawmonster(bestiaire &b, int nbm){
    for(int i = 0; i < nbm; i++){
        drawmonster(b.monstertab[i]);
    }
}

void updategroupmonster(bestiaire &b, int nbm){
    for(int i = 0; i < nbm; i++){
        updatemonster(b.monstertab[i]);
    }
}

/////////////////////////§SET_MAP§////////////////////////////////

void drawMap2d(Data& d , monster m , player p)
{

    int x, y;

    for(y=0; y<mapY; y++) {
        for(x=0; x<mapX; x++) {
            if(mapa[x][y]==1) {
                image_draw(d.minimap, x*l, y*l, l, l);
            }

            if(mapa[x][y]==5) {
                    color(0, 0, 255);
                rectangleFill( x*l, y*l, x*l + l, y*l + l);

            }
        }
    }
}

int heuristic(Complex a, Complex b) {

    return abs(a.x - b.x) + abs(a.y - b.y);
}

// La fonction 'Pathfinding' calcule le chemin optimal pour que le monstre atteigne le joueur
void Pathfinding(GroupNode& groupNode, monster& m, player p) {

    // On récupère les coordonnées entières du monstre et du joueur, en divisant par 10 pour obtenir la position de la case dans la grille
    Complex coMonstre = make_Complex(floor(m.monsterpos.x/10), floor(m.monsterpos.y/10));
    Complex coPlayer = make_Complex(floor(p.playerpos.x/10), floor(p.playerpos.y/10));

    Complex cheminMinTemp, Case;

    groupNode.tabNode[0].f  = heuristic(coPlayer, coMonstre);

    int MinValue = groupNode.tabNode[0].f +2 ;
    Case = coMonstre;


    for (int i = Case.x - 1; i <= Case.x + 1; i++) {
        for (int j = Case.y - 1; j <= Case.y + 1; j++) {

            // On incrémente le nombre de nœuds explorés
            groupNode.nbrNode++;

            // Si la case est un obstacle, on définit le coût 'g' à 100, sinon on le met à 0 (pour faire en sorte qu'il évite les murs, ici on ne regarde pas le chemin le plus optimal avec les couts de diagonales par exemple)
            if (mapa[i][j] == 1) {
                groupNode.tabNode[groupNode.nbrNode].g = 100;
            } else {
                groupNode.tabNode[groupNode.nbrNode].g = 0;
            }

            // On calcule la distance de Manhattan entre le joueur et la case actuelle
            groupNode.tabNode[groupNode.nbrNode].h = heuristic(coPlayer, make_Complex((float) i, (float) j));

            // On calcule la valeur totale 'f' pour la case actuelle en ajoutant 'g' et 'h'
            groupNode.tabNode[groupNode.nbrNode].f = groupNode.tabNode[groupNode.nbrNode].h + groupNode.tabNode[groupNode.nbrNode].g;

            // On stocke les coordonnées de la case actuelle dans la structure 'node'
            groupNode.tabNode[groupNode.nbrNode].fils = make_Complex((float) i , (float) j);

            // Si la valeur 'f' de la case actuelle est inférieure à la valeur minimale actuelle et supérieure à zéro, on la met à jour
            if (groupNode.tabNode[groupNode.nbrNode].f <= MinValue && groupNode.tabNode[groupNode.nbrNode].f > 0) {
                cheminMinTemp = groupNode.tabNode[groupNode.nbrNode].fils;
                MinValue = groupNode.tabNode[groupNode.nbrNode].f;
            }
        }
    }
    // On réinitialise le nombre de nœuds explorés pour la prochaine node
    groupNode.nbrNode = 0;
    Case = cheminMinTemp;

    //mise à jour de la postion du mosntre
    m.monsterpos.x = 10 * cheminMinTemp.x + l/2;
    m.monsterpos.y = 10 * cheminMinTemp.y + l/2;

}

int main(int, char**)
{
    // Initialisation de la fenêtre
    winInit("Raycasting", dimh, dimw);
    backgroundColor(0, 0, 12);

    // Initialisation des structures de données
    player p;
    Vecteur v;
    Data d;
    monster m;
    bestiaire b;
    GroupNode groupNode;

    int r = 1000; // nombre d'interpolation;
    int nbm = 15; // Nombre de monstres dans le groupe

    // Initialisation du joueur, du monstre et de l'image
    initmonster(m);
    initplayer(p);
    Initimage(d);

    // Initialisation du temps de début de jeu
    float time = elapsedTime();

    // Initialisation de la durée de jeu
    float playTime;

    // Variable qui indique si le jeu est terminé
    bool stop = false;

    // Boucle principale du jeu
    while(!stop) {
        // Efface la fenêtre
        winClear();

        // Si le joueur n'est pas mort
        if(!PlayerDead(p)) {
            // Calcul du temps de jeu
            playTime = elapsedTime();

            // Affichage du score
            print(20, 700, "Score: ");
            print(120, 700, playTime);

            // Déplacement du joueur et affichage des murs
            move_player(p);
            wall_draw(p, v, d, m, r);
        }
        else {
            // Affichage du score en fin de partie
            print(dimh/2 - 100, 600, "Score: ");
            print(dimh/2, 600, playTime);

            // Message de félicitations ou de défaite en fonction du score
            if(playTime < 20 )
                print(dimh/2 - 100, 580, "TROP NUL!");
            else
                print(dimh/2 - 100, 580, "TRES BIEN!");

            // Affichage de l'image de fin de partie
            image_draw(d.mort, 0, 0, dimh, dimw);
        }

        // Gestion du déplacement des monstres avec le pathfinding
        if(elapsedTime() - time > speedMonster) {
            Pathfinding(groupNode, m, p);
            time =  elapsedTime();
        }

        // Gestion des collisions avec le joueur
        player_collision(p);

        // Mise à jour du groupe de monstres
        //updategroupmonster(b, nbm);

        // Calcul de la vue du monstre
        monster_view(m);

        // Déplacement des monstres
        //deplacement_monstre(m);

        // Dessin de l'image de fond et de la carte 2D
        drawimage(d);
        drawMap2d(d, m, p);

        // Dessin des rayons de tir, des monstres et du joueur
        draw_rayon_tir(p, v, r);
        //groupdrawmonster(b, nbm);
        drawmonster(m);
        drawplayer(p);

        // Affichage de la fenêtre
        stop = winDisplay();
    }

    // Fermeture de la fenêtre
    winQuit();
    return 0;
}
