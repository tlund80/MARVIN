#include <one_shot_learning/EstimationResult.hpp>

#include <QDir>
#include <QTextStream>
#include <QDateTime>

namespace dti{
namespace one_shot_learning {

EstimationResult::EstimationResult()
{

}

EstimationResult::~EstimationResult()
{

}

void EstimationResult::addToLog(QString fileName, bool append)
{
  // Add to log
  
  //*******************Data************************//
  // 1. time and date for the estimation
  // 2. error;
  // 3. inlier;
  // 4. model name;
  // 5. id;
  // 6. transformation;
  //
  //**********************************************//
  
  QDir dir;
 // QString path = dir.absolutePath();
  QDir matlabDir = QDir("matlab");
  if(matlabDir.exists())
  {
    
    QStringList folders = matlabDir.entryList(QDir::NoDotAndDotDot | 
					       QDir::Dirs,
					       QDir::Name);//(QDir::Filter::Files,QDir::SortFlag::NoSort)

						
    if(folders.length() > 0)
    {
       
	  QDir subDir = QDir("matlab/pose_estimation_results/");
	  QStringList files = subDir.entryList(QDir::NoDotAndDotDot | 
						 QDir::Files,
						 QDir::Name);//(QDir::Filter::Files,QDir::SortFlag::NoSort)
	
	  QString txt_path;
      	  if(fileName.endsWith(".txt"))
	  {
	     txt_path = subDir.absolutePath() + "/" + fileName;
	    
	  }else
	  {
	     txt_path = subDir.absolutePath() + "/" + fileName + ".txt";
	     fileName.append(".txt");
	  }
      	 
	  //std::cout << txt_path.toStdString() << std::endl;
	  //std::cout << fileName.toStdString() << std::endl;
	  
	  QFile file(txt_path);
	  
	  //get current date and time
	  QDateTime dateTime = QDateTime::currentDateTime();
	  QString dateTimeString = dateTime.toString();
	    
      	 if(files.contains(fileName,Qt::CaseSensitive))
	 { //Open file and append result
	 
	   file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
	   QTextStream out(&file);
	    out << dateTimeString + " \t" + _scene_name + " \t" + QString::number(_scene_instances) + " \t" + _name +  " \t" + QString::number(_inlier) + " \t" + QString::number(_error) + " \t" + 
		QString::number(_transformation.translation.x) + " " + QString::number(_transformation.translation.y) + " " + QString::number(_transformation.translation.z) + "  \t" +
	        QString::number(_transformation.rotation.w) + " " + QString::number(_transformation.rotation.x) + " " + QString::number(_transformation.rotation.y) + " "  + QString::number(_transformation.rotation.z) + " \n";
	   
	   file.close();
	  
	 }else
	 { // Create file
	   
	    file.open(QIODevice::WriteOnly | QIODevice::Text);
	   QTextStream out(&file);
	   out << "Date and time               |  Scene Name   |  Instances |  Object name  | Inlier Fraction |  Error  |     T(| x | y | z |)     |    R(| w | x | y | z |)   |\n";
	   out << dateTimeString + " \t" + _scene_name + " \t" + QString::number(_scene_instances) + " \t" + _name +  " \t" + QString::number(_inlier) + " \t" + QString::number(_error) + " \t" + 
		QString::number(_transformation.translation.x) + " " + QString::number(_transformation.translation.y) + " " + QString::number(_transformation.translation.z) + "  \t" +
	        QString::number(_transformation.rotation.w) + " " + QString::number(_transformation.rotation.x) + " " + QString::number(_transformation.rotation.y) + " "  + QString::number(_transformation.rotation.z) + " \n";
	   
	   
	   file.close(); 
	 }
       
    }
    else
    {
       QDir().mkdir("pose_estimation_results");
       //QDir().mkdir("grasping_results");  
    }
    
  }else
  {
     std::cout << "Creating matlab directory!!" << std::endl;
     QDir().mkdir("matlab");
     if(matlabDir.exists())
     {
       matlabDir.mkdir("pose_estimation_results");
      // matlabDir.mkdir("grasping_results"); 
       
     }else std::cout << "Could not create folders for storing the results!!" << std::endl;
   
      
  }
  
  
}

}
}