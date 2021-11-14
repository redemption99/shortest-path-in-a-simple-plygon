#include <SFML/Graphics.hpp>
#include "geometry.h"
#include <bits/stdc++.h>
#include <unistd.h>

using namespace std;

const int stage_delay = 3, step_delay = 1;

void esc(char* error)
{
    printf("%s\n", error);
    exit(0);
}

// O(n^3)
vector<geometry::Triangle> Triangulation(const vector<geometry::Point>& poly)
{
    vector<geometry::Point> points = poly;
    vector<geometry::Triangle> triangles;

    int n;
    while ((n = points.size()) > 3)
    {
        for (int i = 0; i < n; i++)
        {
            geometry::Point a, b, c;
            a = points[i];
            b = points[(i+1) % n];
            c = points[i-1>=0 ? i-1 : n-1];

            if (Angle(a, b, c) >= geometry::pi)
                continue;

            geometry::Triangle ti({a, b, c});

            bool ear = true;

            for (int j = 0; j < poly.size(); j++)
                if (a.idx != poly[j].idx && b.idx != poly[j].idx && c.idx != poly[j].idx)
                    if (InsideTriangle(ti, poly[j]))
                    {
                        ear = false;
                        break;
                    }

            if (ear)
            {
                ti.idx = triangles.size();
                triangles.push_back(ti);
                points.erase(points.begin() + i);
                break;
            }
        }
    }

    triangles.push_back({points[0], points[1], points[2]});

    for (int i = 0; i+1 < triangles.size(); i++)
        for (int j = i+1; j < triangles.size(); j++)
        {
            int cnt = 0;
            for (int i2 = 0; i2 < 3; i2++)
                for (int j2 = 0; j2 < 3; j2++)
                    if (triangles[i].points[i2].idx == triangles[j].points[j2].idx)
                        cnt++;

            if (cnt == 2)
            {
                triangles[i].neigh.push_back(&triangles[j]);
                triangles[j].neigh.push_back(&triangles[i]);
            }
        }

    return triangles;
}

geometry::Triangle* FindTriangle(vector<geometry::Triangle> &triang, geometry::Point p)
{
    for (int i = 0; i < triang.size(); i++)
        if (InsideTriangle(triang[i], p))
            return &triang[i];

    return 0;
}

void dfs(vector<geometry::Triangle*> &path, geometry::Triangle* Di, geometry::Triangle* Dprev, geometry::Triangle* DT)
{
    path.push_back(Di);
    if (Di == DT)
        return;
    for (int i = 0; i < (*Di).neigh.size(); i++)
        if ((*Di).neigh[i] != Dprev)
        {
            dfs(path, (*Di).neigh[i], Di, DT);
            if (path.back() == DT)
                return;
        }
    path.pop_back();
}

vector<geometry::Triangle*> FindPathInDualGraph(geometry::Triangle* DS, geometry::Triangle* DT)
{
    vector<geometry::Triangle*> path;

    dfs(path, DS, 0, DT);

    return path;
}

void PrepareSleeve(vector<geometry::Triangle*> &D, geometry::Point S, geometry::Point T,
                   geometry::Triangle &ts, geometry::Triangle &tt)
{
    // Replace point from first triangle that is not on first diagonal with S
    for (int i = 0; i < 3; i++)
    {
        bool onDiag = false;
        for (int j = 0; j < 3; j++)
            if (ts.points[i].idx == D[1]->points[j].idx)
            {
                onDiag = true;
                break;
            }
        if (!onDiag)
        {
            ts.points[i] = S;
            break;
        }
    }
    D[0] = &ts;


    // Replace point from last triangle that is not on last diagonal with T
    int n = D.size();
    for (int i = 0; i < 3; i++)
    {
        bool onDiag = false;
        for (int j = 0; j < 3; j++)
            if (tt.points[i].idx == D[n-2]->points[j].idx)
            {
                onDiag = true;
                break;
            }
        if (!onDiag)
        {
            tt.points[i] = T;
            break;
        }
    }
    D[n-1] = &tt;

}

vector<geometry::LineSegment> FindDiagonals(vector<geometry::Triangle*> D)
{
    vector<geometry::LineSegment> diagonals;
    for (int i = 0; i+1 < D.size(); i++)
    {
        for (int i1 = 0; i1 < 3; i1++)
        {
            bool inBoth = false;
            for (int i2 = 0; i2 < 3; i2++)
                if (D[i]->points[i1].idx == D[i+1]->points[i2].idx)
                {
                    inBoth = true;
                    break;
                }
            if (!inBoth)
            {
                diagonals.push_back({D[i]->points[(i1+1)%3], D[i]->points[(i1+2)%3]});
            }
        }
    }

    return diagonals;
}

void FindChains(vector<geometry::Triangle*> &D, vector<geometry::LineSegment> &d,
                vector<geometry::Point> &l, vector<geometry::Point> &r, geometry::Point &S, geometry::Point &T)
{
    l.push_back(S);
    r.push_back(S);
    if (Turn(S, d[0].a, d[0].b) < 0)
    {
        l.push_back(d[0].b);
        r.push_back(d[0].a);
    }
    else
    {
        l.push_back(d[0].a);
        r.push_back(d[0].b);
    }

    for (int i = 1; i < d.size(); i++)
    {
        if (d[i].b.idx == r.back().idx)
        {
            l.push_back(d[i].a);
            r.push_back(d[i].b);
        }
        else
        {
            l.push_back(d[i].b);
            r.push_back(d[i].a);
        }
    }
    l.push_back(T);
    r.push_back(T);
}

void FindShortestPaths(vector<geometry::Point> &points, vector<int> &prev, vector<int> &order,
                       vector<geometry::Point> &l, vector<geometry::Point> &r,
                       geometry::Point &S, geometry::Point &T)
{
    prev.resize(points.size(), -2);
    prev[S.idx]= -1;
    prev[l[1].idx] = S.idx;
    order.push_back(l[1].idx);
    prev[r[1].idx] = S.idx;
    order.push_back(r[1].idx);

    int v = S.idx;

    for (int i = 2; i < l.size(); i++)
    {
        if (l[i].idx == l[i-1].idx)
        {
            int z = r[i].idx;
            order.push_back(z);
            int w = r[i-1].idx;
            while (w != v)
            {
                if (Turn(points[prev[w]], points[w], points[z]) > 0)
                {
                    prev[z] = w;
                    break;
                }
                w = prev[w];
            }
            if (prev[z] != -2)
                continue;
            w = l[i-1].idx;
            while (w != v)
            {
                if (Turn(points[prev[w]], points[w], points[z]) < 0)
                {
                    prev[z] = w;
                    v = w;
                    break;
                }
                w = prev[w];
            }
            if (prev[z] != -2)
                continue;
            prev[z] = v;
        }
        else
        {
            int z = l[i].idx;
            order.push_back(z);
            int w = l[i-1].idx;
            while (w != v)
            {
                if (Turn(points[prev[w]], points[w], points[z]) < 0)
                {
                    prev[z] = w;
                    break;
                }
                w = prev[w];
            }
            if (prev[z] != -2)
                continue;
            w = r[i-1].idx;
            while (w != v)
            {
                if (Turn(points[prev[w]], points[w], points[z]) > 0)
                {
                    prev[z] = w;
                    v = w;
                    break;
                }
                w = prev[w];
            }
            if (prev[z] != -2)
                continue;
            prev[z] = v;
        }
    }
}


int main()
{
    printf("Polygon is given by an array of points in counterclockwise order.\n");
    FILE *f = fopen("testcases/test1.txt", "r");
    int n;
    fscanf(f, "%d", &n);

    vector<geometry::Point> poly;

    for (int i = 0; i < n; i++)
    {
        double x, y;
        fscanf(f, "%lf%lf", &x, &y);
        poly.push_back({x, y, i});
    }

    geometry::Point S, T;
    fscanf(f, "%lf%lf", &S.x, &S.y);
    S.idx = n;
    fscanf(f, "%lf%lf", &T.x, &T.y);
    T.idx = n+1;

    vector<geometry::Point> points = poly;
    points.push_back(S);
    points.push_back(T);

    if (IsSelfIntersectingPoly(poly))
        esc("Polygon is self-intersecting.");



    printf("Triangulation started... ");
    // Triangulate polygon
    vector<geometry::Triangle> triang = Triangulation(poly);
    printf("Triangulation done\n");

    // Find triangles that contain S and T
    geometry::Triangle* DS = FindTriangle(triang, S);
    geometry::Triangle* DT = FindTriangle(triang, T);

    if (!DS)
        esc("S is not inside polygon!");
    if (!DT)
        esc("T is not inside polygon!");


    if (DS == DT)
        esc("U istom su trouglicu prika xD");

    printf("Finding path from S to T in dual graph... ");
    // Find a path D from S to T in dual graph of triangulation
    vector<geometry::Triangle*> D = FindPathInDualGraph(DS, DT);
    printf("Path from S to T found\n");

    geometry::Triangle ts = *D[0];
    geometry::Triangle tt = *D[D.size()-1];
    PrepareSleeve(D, S, T, ts, tt);

    printf("Finding diagonals d... ");
    vector<geometry::LineSegment> d = FindDiagonals(D);
    printf("Diagonals found\n");

    printf("Finding left and right chain... ");
    vector<geometry::Point> l, r;
    FindChains(D, d, l, r, S, T);
    printf("Left and right chain found\n");

    printf("Left chain (clockwise): ");
    for (int i = 0; i < l.size(); i++)
        printf("%d ", l[i].idx);
    printf("\n");
    printf("Right chain (counterclockwise): ");
    for (int i = 0; i < r.size(); i++)
        printf("%d ", r[i].idx);
    printf("\n");

    printf("Finding shortest paths... ");
    vector<int> prev, order;
    FindShortestPaths(points, prev, order, l, r, S, T);
    printf("Shortest paths found\n");
    for (int i = 0; i < points.size(); i++)
        if (prev[i] != -2)
            printf("prev[%d] = %d\n", i, prev[i]);
    printf("Discovery order: ");
    for (int i = 0; i < order.size(); i++)
        printf("%d ", order[i]);
    printf("\n");

    //TODO: Improve animation

    /*
    0 - poligon, S i T
    1 - triangulacija
    2 - najdemo DS i DT (trouglove u kojima se nalaze S i T)
    3 - nadjemo D (putanju od DS do DT)
    4 - nadjemo sleeve (unija D sa isecenim DS' i DT')
    5 - nadjemo levi i desni lanac
    6 - za svaku tacku na lancu nadjemo minimalnu putanju
    7 - izdvojimo finalnu putanju
    */

    int stage = 0;

    sf::RenderWindow window(sf::VideoMode(1200, 700), "Shortest path in a Simple Polygon");


    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        sf::Font font;
        font.loadFromFile("Courier.ttf");

        if (stage >= 1 && stage <= 3)
        {
            for (auto t : triang)
            {
                sf::ConvexShape convex;
                convex.setPointCount(3);
                convex.setPoint(0, sf::Vector2f(t.a.x, t.a.y));
                convex.setPoint(1, sf::Vector2f(t.b.x, t.b.y));
                convex.setPoint(2, sf::Vector2f(t.c.x, t.c.y));
                convex.setFillColor(sf::Color(0, (int)(t.b.x+t.a.y) % 256, (int)(t.c.x+t.c.y) %256));
                window.draw(convex);
            }
        }

        if (stage >= 0)
        {
            sf::ConvexShape convex;
            convex.setPointCount(poly.size());
            for (int i = 0; i < poly.size(); i++)
            {
                convex.setPoint(i, sf::Vector2f(poly[i].x, poly[i].y));

                sf::CircleShape shapeP(2.f);
                shapeP.setFillColor(sf::Color::White);
                shapeP.setPosition(poly[i].x-1, poly[i].y-1);
                window.draw(shapeP);

                sf::Text text;
                text.setFont(font);
                text.setCharacterSize(20);
                text.setColor(sf::Color::White);
                text.setStyle(sf::Text::Bold);
                text.setString(to_string(i));
                text.setPosition(sf::Vector2f(poly[i].x+3, poly[i].y+3));
                window.draw(text);
            }
            convex.setFillColor(sf::Color::Transparent);
            convex.setOutlineThickness(2);
            convex.setOutlineColor(sf::Color::White);
            window.draw(convex);

            sf::CircleShape shapeS(3.f);
            shapeS.setFillColor(sf::Color::White);
            shapeS.setPosition(S.x, S.y);
            window.draw(shapeS);

            sf::Text textS;
            textS.setFont(font);
            textS.setCharacterSize(20);
            textS.setColor(sf::Color::White);
            textS.setStyle(sf::Text::Bold);
            textS.setString("S");
            textS.setPosition(sf::Vector2f(S.x+3, S.y+3));
            window.draw(textS);

            sf::CircleShape shapeT(3.f);
            shapeT.setFillColor(sf::Color::White);
            shapeT.setPosition(T.x, T.y);
            window.draw(shapeT);

            sf::Text textT;
            textT.setFont(font);
            textT.setCharacterSize(20);
            textT.setColor(sf::Color::White);
            textT.setStyle(sf::Text::Bold);
            textT.setString("T");
            textT.setPosition(sf::Vector2f(T.x+3, T.y+3));
            window.draw(textT);


            if (stage <= 1)
            {
                window.display();
                sleep(stage_delay);
            }
        }

        if (stage == 2 || stage == 3)
        {
            geometry::Triangle t = *DS;
            sf::ConvexShape convexS;
            convexS.setPointCount(3);
            convexS.setPoint(0, sf::Vector2f(t.a.x, t.a.y));
            convexS.setPoint(1, sf::Vector2f(t.b.x, t.b.y));
            convexS.setPoint(2, sf::Vector2f(t.c.x, t.c.y));
            convexS.setFillColor(sf::Color::Transparent);
            convexS.setOutlineThickness(3);
            convexS.setOutlineColor(sf::Color::Red);
            window.draw(convexS);

            t = *DT;
            sf::ConvexShape convexT;
            convexT.setPointCount(3);
            convexT.setPoint(0, sf::Vector2f(t.a.x, t.a.y));
            convexT.setPoint(1, sf::Vector2f(t.b.x, t.b.y));
            convexT.setPoint(2, sf::Vector2f(t.c.x, t.c.y));
            convexT.setFillColor(sf::Color::Transparent);
            convexT.setOutlineThickness(3);
            convexT.setOutlineColor(sf::Color::Red);
            window.draw(convexT);

            if (stage == 2)
            {
                window.display();
                sleep(stage_delay);
            }
        }

        if (stage == 3)
        {
            sf::ConvexShape convex[D.size()];

            for (int i = 1; i+1 < D.size(); i++)
            {
                for (int j = 1; j <= i; j++)
                {
                    geometry::Triangle t = *D[j];
                    convex[i].setPointCount(3);
                    convex[i].setPoint(0, sf::Vector2f(t.a.x, t.a.y));
                    convex[i].setPoint(1, sf::Vector2f(t.b.x, t.b.y));
                    convex[i].setPoint(2, sf::Vector2f(t.c.x, t.c.y));
                    convex[i].setFillColor(sf::Color::Transparent);
                    convex[i].setOutlineThickness(3);
                    convex[i].setOutlineColor(sf::Color::Red);
                    window.draw(convex[i]);

                }
                window.display();
                sleep(step_delay);
            }
            sleep(stage_delay);
        }

        if (stage == 4)
        {
            sf::ConvexShape convex[D.size()];

            for (int i = 0; i < D.size(); i++)
            {
                geometry::Triangle t = *D[i];
                convex[i].setPointCount(3);
                convex[i].setPoint(0, sf::Vector2f(t.a.x, t.a.y));
                convex[i].setPoint(1, sf::Vector2f(t.b.x, t.b.y));
                convex[i].setPoint(2, sf::Vector2f(t.c.x, t.c.y));
                convex[i].setFillColor(sf::Color::Transparent);
                convex[i].setOutlineThickness(3);
                convex[i].setOutlineColor(sf::Color::Red);
                window.draw(convex[i]);
            }
            window.display();
            sleep(stage_delay);
        }

        if (stage == 5 || stage == 6)
        {
            for (int i = 1; i < l.size(); i++)
            {
                if (l[i].idx != l[i-1].idx)
                {
                    sf::ConvexShape convex;
                    convex.setPointCount(4);
                    convex.setPoint(0, sf::Vector2f(l[i].x, l[i].y));
                    convex.setPoint(1, sf::Vector2f(l[i-1].x, l[i-1].y));
                    convex.setPoint(2, sf::Vector2f(l[i-1].x+2, l[i-1].y));
                    convex.setPoint(3, sf::Vector2f(l[i].x+2, l[i].y));

                    convex.setFillColor(sf::Color(255, 132, 0));
                    convex.setOutlineThickness(2);
                    convex.setOutlineColor(sf::Color(255, 132, 0));
                    window.draw(convex);
                }
            }
            for (int i = 1; i < r.size(); i++)
            {
                if (r[i].idx != r[i-1].idx)
                {
                    sf::ConvexShape convex;
                    convex.setPointCount(4);
                    convex.setPoint(0, sf::Vector2f(r[i].x, r[i].y));
                    convex.setPoint(1, sf::Vector2f(r[i-1].x, r[i-1].y));
                    convex.setPoint(2, sf::Vector2f(r[i-1].x+2, r[i-1].y-2));
                    convex.setPoint(3, sf::Vector2f(r[i].x+2, r[i].y-2));

                    convex.setFillColor(sf::Color(0, 255, 76));
                    convex.setOutlineThickness(2);
                    convex.setOutlineColor(sf::Color(0, 255, 76));
                    window.draw(convex);
                }
            }

            if (stage == 5)
            {
                window.display();
                sleep(stage_delay);
            }
        }

        if (stage == 6)
        {
            for (int i = 0; i < order.size(); i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    sf::ConvexShape convex;
                    convex.setPointCount(4);
                    int idx1 = order[j];
                    int idx2 = prev[order[j]];
                    convex.setPoint(0, sf::Vector2f(points[idx1].x, points[idx1].y));
                    convex.setPoint(1, sf::Vector2f(points[idx2].x, points[idx2].y));
                    convex.setPoint(2, sf::Vector2f(points[idx2].x+2, points[idx2].y-2));
                    convex.setPoint(3, sf::Vector2f(points[idx1].x+2, points[idx1].y-2));

                    convex.setFillColor(sf::Color::Red);
                    convex.setOutlineThickness(2);
                    convex.setOutlineColor(sf::Color::Red);

                    window.draw(convex);
                }
                window.display();
                sleep(step_delay);
            }
            sleep(stage_delay);
        }

        if (stage == 7)
        {
            int idx = T.idx;
            while (prev[idx] >= 0)
            {
                sf::ConvexShape convex;
                convex.setPointCount(4);
                int idx2 = prev[idx];
                convex.setPoint(0, sf::Vector2f(points[idx].x, points[idx].y));
                convex.setPoint(1, sf::Vector2f(points[idx2].x, points[idx2].y));
                convex.setPoint(2, sf::Vector2f(points[idx2].x+2, points[idx2].y-2));
                convex.setPoint(3, sf::Vector2f(points[idx].x+2, points[idx].y-2));

                convex.setFillColor(sf::Color::Yellow);
                convex.setOutlineThickness(2);
                convex.setOutlineColor(sf::Color::Yellow);

                window.draw(convex);

                idx = prev[idx];
            }
            window.display();
            sleep(stage_delay);
            continue;
        }

        if (stage > 7)
        {
            sleep(10);
            break;
        }

        stage++;
    }

    return 0;
}

