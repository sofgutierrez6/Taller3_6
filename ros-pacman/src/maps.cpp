#include "pacman/maps.h"

Maps::Maps(QWidget* parent)
{
    
}

QByteArray Maps::File2ArrayMap(QString fileName, int& colsText, int& rowsText)
{
    //Carga archivo .lay
    QFile file(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/" + fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return nullptr;
    
    //Lectura datos mapa
    QByteArray text;
    QByteArray line;
    bool firstLine = false;
    while (!file.atEnd()) {
        line = file.readLine();
	for(int i = 0; i < line.size(); i++)
	{
	    if(line[i] != '\n')
		text.append(line[i]);
	}
	
	if(!firstLine)
	{
	    colsText = line.size() - 1;
	    firstLine = true;
	}
    }
    rowsText = (text.size()/(colsText));
 	
    //Hacer que el mapa tenga siempre un centro cuando se convierte a coordenadas
    if(rowsText%2 == 0)
    {
		for(int i = 0; i < colsText; i++)
		    text.append('%');
		rowsText++;
    }
    if(colsText%2 == 0)
    {
		for(int i = 0; i < rowsText; i++)
		    text.insert( (i+1)*colsText + i, '%');
		colsText++;
    }	
 	   
    return text;
}

void Maps::CreateObstaclesArray(QByteArray text, int colsText, int rowsText)
{
    cols = colsText*BLOCK_SIZE;
    rows = rowsText*BLOCK_SIZE;
    mObstacles = new bool[(rows)*(cols)];
    
    //Inicializacion matriz obstaculos
    for(int i = 0; i < rows; i++)
	for(int j = 0; j < cols; j++)
	    mObstacles[i*cols + j] = false;
    
    pPacman = new QVector<int>;
    pGhosts = new QVector<int>;
    pCookies = new QVector<int>;
    pBonus = new QVector<int>;
    pObstacles = new QVector<int>;
    
    //Construccion matriz de obstaculos
    for(int i = 0; i < rowsText; i++)
    {
	for(int j = 0; j < colsText; j++)
	{
	    //Obstaculos
	    if(text[i*colsText + j] == '%') 
	    {
		for(int k = (i*BLOCK_SIZE); k < ((i*BLOCK_SIZE) + BLOCK_SIZE); k++)
		{
		    for(int l = (j*BLOCK_SIZE); l < ((j*BLOCK_SIZE) + BLOCK_SIZE); l++)
		    {
			mObstacles[k*cols + l] = true;
		    }
		}
		pObstacles->append(i);
		pObstacles->append(j);
	    }
	    //Galletas
	    else if(text[i*colsText + j] == '.') 
	    {
		pCookies->append(i);
		pCookies->append(j);
	    }
	    //Bonos
	    else if(text[i*colsText + j] == 'o') 
	    {
		pBonus->append(i);
		pBonus->append(j);
	    }
	    //Pacman
	    else if(text[i*colsText + j] == 'P')
	    {
		pPacman->append(i);
		pPacman->append(j);	
	    } 
	    //Ghosts
	    else if(text[i*colsText + j] == 'G')
	    {
		pGhosts->append(i);
		pGhosts->append(j);
	    } 
	}
    }
}

void Maps::PrintObstaclesArray()
{
    //Imprimir matriz obstaculos
    cout << "------------------------------------------" <<endl;
    for(int i = 0; i < rows; i++)
    {
	for(int j = 0; j < cols; j++)
	    cout << mObstacles[i*cols + j] << "";
	cout << endl;
    }
    cout << "------------------------------------------" <<endl;
}

void Maps::CreateImageFromObstaclesArray()
{
    image = new QImage(cols, rows, QImage::Format_RGB32);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            QRgb value;
            if(mObstacles[i*cols + j]) //Obstacles
            {
                value = qRgb(255,255,255); //White
		image->setPixel(j, i, value);
            }
            else //No Obstacles
            {
		value = qRgb(0,0,0); //Black
		image->setPixel(j, i, value);
            }
      }
   }
}

void Maps::SaveImage(QString fileName)
{
    //Guardar Imagen
   QFile fileMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/textures/" + fileName);
   if(!fileMap.exists())//Si existe la imagen no se guarda
   {
      if(fileMap.open(QIODevice::WriteOnly | QIODevice::Truncate))
	  image->save(&fileMap, "PNG");
      else
	  cout << "No se pudo abrir" << endl;
   }
}

int Maps::getWidth()
{
    return cols;
}

int Maps::getHeight()
{
    return rows;
}


void Maps::CreateMap(QString nameMap)
{
    //Lectura de archivo
    int colsText, rowsText;
    QByteArray text = File2ArrayMap(nameMap + ".lay", colsText, rowsText);
    
    //Creacion matriz de obstaculos
    CreateObstaclesArray(text, colsText, rowsText);
    
    //Impresion matriz obstaculos
    //printObstaclesArray();
    
    //Creacion imagen
    CreateImageFromObstaclesArray();

    //Guardar imagen
    SaveImage(nameMap + ".png");
    
    emit SendMapData(BLOCK_SIZE, BLOCK_SIZE, image, mObstacles, pPacman, pGhosts, pCookies, pBonus, pObstacles, rowsText-1, colsText-1);
}