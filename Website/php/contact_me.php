<?php
// Check for empty fields
if(empty($_POST['name'])  		||
   empty($_POST['email']) 		||
   empty($_POST['phone']) 		||
   empty($_POST['message'])	||
   !filter_var($_POST['email'],FILTER_VALIDATE_EMAIL))
   {
	echo "No arguments Provided!";
	return false;
   }

$name = $_POST['first_name'];
$last = $POST['last_name']
$email_address = $_POST['email'];
$phone = $_POST['telephone'];
$message = $_POST['comments'];


$to = '18212@student.bbc.qld.edu.au; 18743@student.bbc.qld.edu.au';
$email_subject = "FGB Robotics Contact Form:  $name $last";
$email_body = "You have received a new message from your website contact form.\n\n"."Here are the details:\n\nName: $name\n\nEmail: $email_address\n\nPhone: $phone\n\nMessage:\n$message";
$headers = "From: hulbertf@bigpond.com\n"; // This is the email address the generated message will be from. We recommend using something like noreply@yourdomain.com.
$headers .= "Reply-To: $email_address";
mail($to,$email_subject,$email_body,$headers);
return true;

echo "<script type='text/javascript'>alert('submitted successfully!')</script>";
header( 'Location: http://fgb.bbcrobotics.org' )
?>
