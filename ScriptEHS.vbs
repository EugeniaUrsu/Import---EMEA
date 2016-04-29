const PI           = 3.141593
const CLR_WHITE    = 16777215
const CLR_BLACK    = 0
const CLR_RED      = 255
const CLR_LTRED    = 6053119
const CLR_CORAL    = 8388863
const CLR_MAROON   = 128
const CLR_OLIVE    = 32896
const CLR_TEAL     = 8421376
const CLR_GREEN    = 32768
const CLR_SEAGREEN = 8453888
const CLR_LIME     = 65280
const CLR_LTGREEN  = 65408
const CLR_BLUE     = 16711680
const CLR_MIDNIGHT = 8388608
const CLR_SKY      = 16744448
const CLR_PURPLE   = 8388736
const CLR_VIOLET   = 16711808
const CLR_BEIGE    = 33023
const CLR_CYAN     = 16776960
const CLR_MAGENTA  = 16711935
const CLR_YELLOW   = 65535
const CLR_GRAY10   = 15132390
const CLR_GRAY20   = 13487565
const CLR_GRAY30   = 11776947
const CLR_GRAY40   = 10000536
const CLR_GRAY50   = 8421504
const CLR_GRAY60   = 6710886
const CLR_GRAY70   = 5000268
const CLR_GRAY80   = 3355443
const CLR_GRAY90   = 1644825
const CLR_ORANGE   = 26367
const CLR_BROWN    = 13209

const     EDT_NULL = 0
const     EDT_BOOL = 1
const     EDT_ENUM = 2
const     EDT_FLOAT = 3
const     EDT_INT = 4
const     EDT_STRING = 5
const     EDT_LINT = 6
const     EDT_UINT = 7
const     EDT_DOUBLE = 8
const     EDT_TIME = 9
const     EDT_HEX = 10
const     MB_READ_ONLY = 1
const     MB_READ_WRITE = 0

const      TD_IEEEANSI_MOD_INVERSE				= 0
const     	TD_IEEEANSI_INVERSE					= 1
const      TD_IEEEANSI_VERY_INVERSE			= 2
const      TD_IEEEANSI_EXTEMELY_INVERSE		= 3
const     	TD_IEEEANSI_SHORT_TIME_INVERSE		= 4
const      TD_IEC_INVERSE						= 5
const      TD_IEC_VERY_INVERSE					= 6
const      TD_IEC_EXTEMELY_INVERSE				= 7
const      TD_IEC_LONG_TIME_INVERSE			= 8
const     	TD_IEC_SHORT_TIME_INVERSE			= 9
const      NUM_TD_ANSI_IEC_CURVES				= 10
const     	TD_THERM_IT							= 11
const     	TD_THERM_I2T						= 12
const     	TD_THERM_I4T						= 13
const     	TD_THERM_FLAT						= 14

'*****************************************************************************
'
FUNCTION divide(numerator, denominator)
'
'*****************************************************************************

    divide = Empty
    if not IsEmpty(numerator) and not IsEmpty(denominator) then
       on error resume next
    divide  = numerator / denominator
    on error goto 0
    end if
END FUNCTION


'*****************************************************************************
'
FUNCTION setIfEmpty(dstval, srcval)
'
'*****************************************************************************

    IF dstval = Empty THEN
        setIfEmpty = srcval
    ELSE
        setIfEmpty = dstval
    END IF
END FUNCTION


'*****************************************************************************
'
FUNCTION LoadLookupTable(ByRef lut(), ByVal NumCols, ByVal filename)
'
'  This FUNCTION loads a 2 dimensional array with the values from a comma-
'  delimited file.  The array is loaded in the form lut(rows,columns).
'  The provided array must have adequate dimensions to hold the file contents.
'  If it does not a message is displayed.
'
'*****************************************************************************

   DIM textfile, textline
   DIM CommaAt, AtChar
   DIM ErrStr

   Const ForReading = 1

   Set fso = CreateObject("Scripting.FileSystemObject")
   Set textfile = fso.OpenTextFile(filename, ForReading)

   ErrStr = TLang("FUNCTION LoadLookupTable(ByRef lut(), filename)") + Chr(13) + Chr(13) + TLang("The file ") + filename

   '
   ' Count the lines.
   '
   LineNum = 0
   do while textfile.AtEndOfStream <> true
     textline = textfile.ReadLine
     LineNum = LineNum + 1
   loop
   textfile.Close

   redim lut(NumCols, LineNum)

   Set textfile = fso.OpenTextFile(filename, ForReading)
   LineNum = 0
   do while textfile.AtEndOfStream <> true

     '
	 ' Parse the line IF it doesn't begin with a comment.
	 '
     textline = textfile.ReadLine
     IF Left(textline, 1) <> "'" THEN

       LineNum = LineNum + 1

       LineLen = Len(textline)

       VarNum = 0
       AtChar = 1
       do
         CommaAt = Instr(AtChar, textline, ",")
         IF CommaAt <> 0 THEN
           SubStr = Mid(textline, AtChar, CommaAt - AtChar)
         ELSE
           SubStr = Mid(textline, AtChar, LineLen)
         END IF
         VarNum = VarNum + 1

         IF VarNum > NumCols THEN
           msgbox ErrStr + TLang(" has too many columns for the provided lookup table array.")
           exit FUNCTION
         END IF

         Lut(VarNum, LineNum) = CDbl(SubStr)

         AtChar = CommaAt + 1
       loop while ((CommaAt <> 0) AND (CommaAt <> Empty))

     END IF
   loop


   textfile.Close

END FUNCTION


'*****************************************************************************
'
FUNCTION LookupValue(ByRef lut(), ByVal SearchColumn, ByVal SearchValue, ByVal LookupColumn)
'
'  This FUNCTION searches the specified column of a 2 dimensional array
'  for a specified value.  If the value is found THEN the value in a
'  specified "lookup" column is returned; otherwise an empty value is
'  returned.
'
'*****************************************************************************

  DIM LutRows, LutCols, RowNum
  DIM ErrStr

  LookupValue = Empty

  ErrStr = TLang("FUNCTION LookupValue(ByRef lut(), SearchColumn, SearchValue, LookupColumn)") + chr(13) + chr(13)
  LutCols = UBound(lut, 1)
  LutRows = UBound(lut, 2)

  IF SearchColumn > LutCols THEN
     msgbox ErrStr + TLang("SearchColumn out of bounds")
  END IF

  IF LookupColumn > LutCols THEN
     msgbox ErrStr + TLang("LookupColumn out of bounds")
  END IF

  FoundAtRow = -1
  for RowNum = 1 to LutRows

    IF lut(SearchColumn, RowNum) = SearchValue THEN
      FoundAtRow = RowNum
      exit for
    END IF
  next

  IF FoundAtRow <> -1 THEN
    LookupValue = lut(LookupColumn, FoundAtRow)
  END IF

END FUNCTION

'*****************************************************************************
'
'SUB LogMsg(Msg)
'
'  use alog() now instead of this
'
'*****************************************************************************
SUB LogMsg(Msg)
END SUB

'*****************************************************************************
'
'SUB SetNdxVar(Name,Ndx,Value)
'
'  This subroutine sets a variable name + _Ndx.  e.g. SetNdxVar("abc",3,4
'  is the same as .abs_3 = 4
'
'*****************************************************************************
SUB SetNdxVar(Name,Ndx,Value)
    with Form
    dim WholeName
    WholeName = Name + "_" + CStr(Ndx)
    Id = .GetDispId(WholeName)
    if Id = -1 then
        .DeclareVar(WholeName)
        Id = .GetDispId(WholeName)
    end if
     call .SetValue(Id,Value)
     end with
 end sub

'*****************************************************************************
'
'SUB SetVar(Name,Ndx,Value)
'
'  This subroutine sets a variable name
'
'*****************************************************************************
SUB SetVar(Name,Value)
    with Form
    Id = .GetDispId(Name)
    if Id = -1 then
        .DeclareVar(Name)
        Id = .GetDispId(Name)
    end if
     call .SetValue(Id,Value)
     end with
 end sub
'*****************************************************************************
'
'SUB SetVarIfEmpty(Name,Value)
'
'  This subroutine sets a variable if the target field is empty
'
'*****************************************************************************
SUB SetVarIfEmpty(Name,Value)
    with Form
    Id = .GetDispId(Name)
    if Id <> -1 then
        Tmp = .GetValue(Id)
        if Tmp = empty then
             call .SetValue(Id,Value)
        end if
    end if
    end with
 end sub
'*****************************************************************************
'
'SUB SetNdxVarIfEmpty(Name,Value,Ndx)
'
'  This subroutine sets a variable if the target field is empty
'
'*****************************************************************************
SUB SetNdxVarIfEmpty(Name,Value,Ndx)
    Name = Name + "_" + CStr(Ndx)
    call SetVarIfEmpty(Name,Value)
 end sub
'*****************************************************************************
'
'FUNCTION GetNdxVar(Name,Ndx)
'
'  This subroutine returns the variable name + _Ndx.  e.g. GetNdxVar("abc",3)
'  is the same as .abs_3
'
'*****************************************************************************
FUNCTION GetNdxVar(Name,Ndx)
    with Form
    dim WholeName
    WholeName = Name + "_" + CStr(Ndx)
    Id = .GetDispId(WholeName)
    if Id = -1 then
        GetNdxVar = Empty
    else
        GetNdxVar = .GetValue(Id)
    end if
    end with
END FUNCTION

'*****************************************************************************
'
'FUNCTION GetVar(Name)
'
'  This subroutine returns the variable name
'
'*****************************************************************************
FUNCTION GetVar(Name)
    with Form
    Id = .GetDispId(Name)
    if Id = -1 then
        GetVar = Empty
    else
        GetVar = .GetValue(Id)
    end if
    end with
END FUNCTION


'
FUNCTION GetTcfEx(ByVal DegC, ByVal TableName, ByVal Field)
'
'*****************************************************************************

    SelectStatementHigh = "SELECT TOP 1 TempC , Mult1556, " + Field + " as [KFactor] FROM " + TableName + _
        " WHERE TempC >= " + qCstr (DegC) + _
        " ORDER BY TempC "

    SelectStatementLow =  "SELECT TOP 1 TempC , Mult1556, " + Field + " as [KFactor] FROM " + TableName + _
        " WHERE TempC <= " + qCstr(DegC) + _
        " ORDER BY TempC DESC "

    KFactor = empty


    '
    ' If a valid temperature to lookup
    '
    IF NOT IsEmpty(DegC) THEN

        '
        ' Get the point above the lookup point
        '

        Form.DeclareVar("db_TempC")
        Form.DeclareVar("db_KFactor")
        Form.DeclareVar("db_Mult1556")
        call Application.QueryLookupTable(Form.FormId, SelectStatementHigh, 0)
        TempHigh = Form.db_TempC
        KHigh    = Form.db_KFactor
        Mult1556 = Form.db_Mult1556

        '
        ' Get the factor below the lookup point
        '
        call Application.QueryLookupTable(Form.FormId, SelectStatementLow, 0)
        TempLow = Form.db_TempC
        KLow    = Form.db_KFactor


        '
        ' If the lookup point was within the table, interpolate the results
        '
        if not IsEmpty(TempLow) and not IsEmpty(TempHigh) then
            DeltaT = TempHigh - TempLow
            DeltaK = KHigh - KLow
            if DeltaT > 0 then
                Ratio = (DegC - TempLow) / DeltaT
                KFactor = KLow + ( DeltaK * Ratio )
            else
                KFactor = KLow
            end if

			 Form.DeclareVar("PdbTempCorrFactor")
            '
            ' Correct to 15.56 degrees if desired
            '
            IF Form.PdbTempCorrFactor = 15.56 THEN
                KFactor = KFactor * Mult1556
            END IF

            '
            ' convert from vt_r4 to string, to vt_r8
            ' to avoid coersion resolution problem.
            '
            StrTcf = CStr(KFactor)
            KFactor = CDbl(StrTcf)
        END IF

    END IF

    GetTcfEx = KFactor


END FUNCTION
'
FUNCTION GetTcf_TC(ByVal DegC, ByVal TableName, ByVal Field)
'
'*****************************************************************************
    with FORM
    SelectStatementHigh = "SELECT TOP 1 TC as [TempC] , " + Field + " as [KFactor] FROM " + TableName + _
        " WHERE TC >= " + qCstr(DegC) + _
        " ORDER BY TC "

    SelectStatementLow =  "SELECT TOP 1 TC as [TempC], " + Field + " as [KFactor] FROM " + TableName + _
        " WHERE TC <= " + qCstr(DegC) + _
        " ORDER BY TC DESC "

    KFactor = empty
	 .DeclareVar("db_TempC")
	 .DeclareVar("db_KFactor")

    '
    ' If a valid temperature to lookup
    '
    IF NOT IsEmpty(DegC) THEN

        '
        ' Get the point above the lookup point
        '

        call Application.QueryLookupTable(.FormId, SelectStatementHigh, 0)
        TempHigh = .db_TempC
        KHigh    = .db_KFactor

        '
        ' Get the factor below the lookup point
        '
        call Application.QueryLookupTable(.FormId, SelectStatementLow, 0)
        TempLow = .db_TempC
        KLow    = .db_KFactor


        '
        ' If the lookup point was within the table, interpolate the results
        '
        if NOT IsEmpty(TempLow) and NOT IsEmpty(TempHigh) then
            DeltaT = TempHigh - TempLow
            DeltaK = KHigh - KLow
            if DeltaT > 0 then
                Ratio = (DegC - TempLow) / DeltaT
                KFactor = KLow + ( DeltaK * Ratio )
            else
                KFactor = KLow
            end if


            '
            ' convert from vt_r4 to string, to vt_r8
            ' to avoid coersion resolution problem.
            '
            StrTcf = CStr(KFactor)
            KFactor = CDbl(StrTcf)
        END IF

    END IF

    GetTcf_TC = KFactor
	 END WITH


END FUNCTION
'*****************************************************************************
'
FUNCTION GetTcf(ByVal DegC, ByVal Letter)
'
'*****************************************************************************
	 Form.DeclareVar("db_Tcf_156")
	 Form.DeclareVar("db_Tcf_o")
    TableName = "LKP_Tcf_" + Letter
    SelectStatement = "select C_O, s_Generation, s_GUID, Tcf_o, Multiplier, Tcf_156 from " + TableName  + " where C_O = " + Cstr(DegC)

    IF IsEmpty(DegC) THEN
        GetTcf = empty
    ELSE
        call Application.QueryLookupTable(Form.FormId, SelectStatement, 0)

        IF Form.PdbTempCorrFactor = 15.56 THEN
            TmpTcf = Form.db_Tcf_156
        ELSE
            TmpTcf = Form.db_Tcf_o
        END IF

        IF IsEmpty(TmpTcf) THEN
            GetTcf = empty
        ELSE

            ' convert from vt_r4 to string, to vt_r8
            ' to avoid coersion resolution problem.
            StrTcf = CStr(TmpTcf)
            GetTcf = CDbl(StrTcf)
        END IF

    END IF

END FUNCTION


FUNCTION AddCommasToNumeric(ByVal NumericString)

	DIM StartPos					' Start of the insert of commas
	DIM	Ndx							' Index into the string array
	DIM	DigitCount					' Number of digits per comma

        DIM NewStr

	DigitCount = 0

	DIM SaveString      			' Original string



	SaveString = NumericString
        NumericString = StripChar(NumericString, ",")

	'
	' Ignore IF the string already has a comma
	'
	IF ( InStr(NumericString, ",") < 1 ) THEN

		NewStr = ""

		StartPos = InStr(NumericString, ".")

		IF( StartPos < 1 ) THEN
			StartPos = Len(NumericString)
       ELSEIF( StartPos = 1 ) THEN
           StartPos = 0
           NewStr = "0" + NumericString
           NumericString = ""
		ELSE
			StartPos = StartPos - 1
           FractionLen = Len(NumericString) - StartPos
           NewStr = Right(NumericString, FractionLen)
		END IF

		'
		' Insert a comma every three characters
		'
		for Ndx=StartPos to 2 step -1

			IF( not IsNumeric(Mid(NumericString, Ndx, 1) ) ) THEN
				NewStr = SaveString
				exit for
			END IF

			DigitCount = DigitCount + 1
			IF( (DigitCount >= 3) AND _
				(Mid(NumericString, Ndx-1, 1) <> "-") AND _
				(Mid(NumericString, Ndx-1, 1) <> "+" )) THEN

               NewStr = Mid(NumericString, Ndx, 1) + NewStr
				NewStr = "," + NewStr
				DigitCount = 0
           ELSE
        	    NewStr = Mid(NumericString, Ndx, 1) + NewStr
			END IF

		next

       IF Len(NumericString) > 0 THEN
           NewStr = Mid(NumericString, 1, 1) + NewStr
       END IF
       AddCommasToNumeric = NewStr
	ELSE
		AddCommasToNumeric = NumericString
	END IF

END FUNCTION


FUNCTION IsNumericEx(ByVal NumericString)

	IsNumericEx = IsNumeric(NumericString)
	IF not IsNumericEx THEN

                NewStr = StripChar(NumericString, ",")
		IsNumericEx = IsNumeric(NewStr)
	END IF

END FUNCTION


FUNCTION StripChar(ByVal StrVal, ByVal CharVal)


		Length = Len(StrVal)
                NewStr = ""

		'
		' Do extra numeric check without commas.
		'
		for Ndx=1 to Length

			AtChar = Mid(StrVal, Ndx, 1)
			IF AtChar <> CharVal THEN
				NewStr = NewStr + AtChar
			END IF
		next

                StripChar = NewStr

END FUNCTION

FUNCTION CheckNTorNA(ByVal value)

    CheckNTorNA = value

    tmp = value
    IF not IsEmpty(tmp) Then

        IF IsNumericEx(tmp) Then
            CheckNTorNA = AddCommasToNumeric(tmp)
        ELSE

            IF StrComp(tmp, "") = 0 THEN

                '
                ' The field is blank.
                '
                CheckNTorNA = ""

            ELSE

                '
                ' If the field is "*" or "NA" THEN use it;
                ' otherwise set it to "NT".
                '
                tmp = UCase(tmp)

                IF StrComp(tmp, "*") = 0 THEN
                    CheckNTorNA = "*"
                ELSE

                    IF StrComp(tmp, "NA") = 0 THEN
                        CheckNTorNA = "NA"
                    ELSE
                        CheckNTorNA = "NT"
                    END IF
                END IF
            END IF
        END IF
    END IF

END FUNCTION

FUNCTION IsBlankString(ByVal value)

    IsBlankString = false

    IF IsEmpty(value) THEN
        IsBlankString = true
    ELSE
        IF StrComp(value, "") = 0 THEN
            IsBlankString = true
        END IF
    END IF

END FUNCTION

'*****************************************************************************
'
FUNCTION QueryForOrgLevels(byVal ActFormId)
'
'*****************************************************************************
    SqlStr = "SELECT * FROM Relay_Org ORDER BY OrgLevel"
    call Application.QueryDatabase(ActFormId, SqlStr, 1)
END FUNCTION

'*****************************************************************************
'
FUNCTION IntToTime(ByVal Time)
'
'*****************************************************************************
    DIM TimeStr
    DIM Hour
    DIM Minute
    DIM Second
    DIM MinuteStr
    DIM SecondStr
    DIM StrPos

    TimeStr = "00:00:00"

    StrPos = Instr(1, CStr(Time), ":", 0)
    Length = Len(CStr(Time))

    IF (StrPos = 0) and (Length > 0) THEN

        Time = CLng(Time)

        Second = Time Mod 60
        Minute = ((Time - Second)/60) Mod 60
        Hour = (Time - Second - (Minute*60)) / 3600

        MinuteStr = CStr(FIX(Minute))
        IF FIX(Minute) < 10 THEN
            MinuteStr = "0" & MinuteStr
        END IF

        SecondStr = CStr(FIX(Second))
        IF FIX(Second) < 10 THEN
            SecondStr = "0" & SecondStr
        END IF

        HourStr = CStr(FIX(Hour))
        IF FIX(Hour) < 10 THEN
            HourStr = "0" & HourStr
        END IF

'       IF Hour < 0 or Hour > 24 THEN
'           msgbox Hour & "  " &  Time
'       END IF
        TimeStr = HourStr & ":" & MinuteStr & ":" & SecondStr

    ELSE

        TimeStr = Time
    END IF

    IntToTime = TimeStr
END FUNCTION
'*****************************************************************************
'
' FUNCTION IsUnbalanced2(v1,v2,Limit)
'
'  This FUNCTION determines if the two values differ more than Limit percent
'
'*****************************************************************************

function IsUnbalanced2(v1,v2,Limit)
    IsUnbalanced2 = false
    if v1 <> Empty and v2 <> Empty then
        Diff = abs(v1-v2)
        Percent = Divide(Diff,abs(v1)) * 100
        if Percent > Limit then
            IsUnbalanced2 = true
        end if
    end if
end function

'*****************************************************************************
'
' FUNCTION IsUnbalanced3(v1,v2,,v3,Limit)
'
'  This FUNCTION determines if any of the three values differ more than Limit
'  percent
'
'*****************************************************************************

function IsUnbalanced3(v1,v2,v3,Limit)
    IsUnbalanced3 = false
    if IsUnbalanced2(v1,v2,Limit) then IsUnbalanced3 = true
    if IsUnbalanced2(v1,v3,Limit) then IsUnbalanced3 = true
    if IsUnbalanced2(v2,v3,Limit) then IsUnbalanced3 = true
end function

'*****************************************************************************
'
' FUNCTION IsUnbalanced6(v1,v2,,v3,Limit)
'
'  This FUNCTION determines if any of the six values differ more than Limit
'  percent
'
'*****************************************************************************
function IsUnbalanced6(v1,v2,v3,v4,v5,v6,Limit)
    IsUnbalanced6 = false

    if IsUnbalanced2(v1,v2,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v1,v3,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v1,v4,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v1,v5,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v1,v6,Limit) then IsUnbalanced6 = true

    if IsUnbalanced2(v2,v3,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v2,v4,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v2,v5,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v2,v6,Limit) then IsUnbalanced6 = true

    if IsUnbalanced2(v3,v4,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v3,v5,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v3,v6,Limit) then IsUnbalanced6 = true

    if IsUnbalanced2(v4,v5,Limit) then IsUnbalanced6 = true
    if IsUnbalanced2(v4,v6,Limit) then IsUnbalanced6 = true

    if IsUnbalanced2(v5,v6,Limit) then IsUnbalanced6 = true

end function

'*****************************************************************************
'
' SUB Concatenate(s1,s2,seperator)
'
'  This sub adds s1 to the end of s2 and inserts seperator if s1 was not empty
'
'*****************************************************************************
sub Concatenate(s1,s2,seperator)
    if len(s1) > 0 and len(s2) > 0 then
        s1 = s1 + seperator
    end if
    s1 = s1 + s2
end sub

'*****************************************************************************
'
' FUNCTION GetNdxVarMid(StrA,Ndx,StrB)
'
'  This function returns the value of a variable with and index in the middle
'
'*****************************************************************************
function GetNdxVarMid(StrA,Ndx,StrB)
    GetNdxVarMid = ""
    Name = StrA + CStr(Ndx) + StrB
    GetNdxVarMid  = GetVar(Name)
end function
'*****************************************************************************
'
' Sub SetNdxVarMid(StrA,Ndx,StrB,Val)
'
'  This function sets the value of a variable with and index in the middle
'
'*****************************************************************************
sub SetNdxVarMid(StrA,Ndx,StrB,Val)
    Name = StrA + CStr(Ndx) + StrB
    Call SetVar(Name,Val)
end sub

'*****************************************************************************
'
' FUNCTION GetNdxVarNu(StrA,Ndx,StrB)
'
'  This function returns the value of an index variable with no underscore
'
'*****************************************************************************
function GetNdxVarNu(Str,Ndx)
    GetNdxVarNu = ""
    Name = Str + CStr(Ndx)
    GetNdxVarNu  = GetVar(Name)
end function
'*****************************************************************************
'
' Sub SetNdxVarMid(StrA,Ndx,StrB,Val)
'
'  This function sets the value of an index var with no underscore
'
'*****************************************************************************
sub SetNdxVarNu(Str,Ndx,Val)
    Name = Str + CStr(Ndx)
    Call SetVar(Name,Val)
end sub
'*****************************************************************************
'
' Function SafeMultiply(x,y)
'
'  This function multiply x and y if not empty and if they are numeric;
'  If not, it will return empty
'
'*****************************************************************************
function SafeMultiply(x,y)
    SafeMultiply = Empty
    if IsNumeric(x) and not IsEmpty(x) and IsNumeric(y) and not IsEmpty(y) then
        SafeMultiply = x * y
    end if
end function
'*****************************************************************************
'
' Function SafeSubtract(x,y)
'
'  This function subtract x and y if not empty and if they are numeric;
'  If not, it will return empty
'
'*****************************************************************************
function SafeSubtract(x,y)
    SafeSubtract = Empty
    if IsNumeric(x) and not IsEmpty(x) and IsNumeric(y) and not IsEmpty(y) then
        SafeSubtract = x - y
    end if
end function
'*****************************************************************************
'
' Function SafeAdd(x,y)
'
'  This function Add x and y if not empty and if they are numeric;
'  If not, it will return empty
'
'*****************************************************************************
function SafeAdd(x,y)
    SafeAdd = Empty
    if IsNumeric(x) and not IsEmpty(x) and IsNumeric(y) and not IsEmpty(y)then
        SafeAdd = x + y
    end if
end function
'*****************************************************************************
'
' Function GetPi()
'
'  returns PI
'*****************************************************************************
function GetPi()
    GetPi = 3.141593
end function
'*****************************************************************************
'
' Function cosd(angle)
'
'  returs the cosine of the anlge in degrees
'
'*****************************************************************************
function cosd(angle)
    cosd = cos( angle * 2 * GetPi() / 360 )
end function
'*****************************************************************************
'
' Function sind(angle)
'
'  returns the sine of the angle in degrees
'
'*****************************************************************************
function sind(angle)
    sind = sin( angle * 2 * GetPi() / 360 )
end function
'*****************************************************************************
'
' Function ParseString(StrSrc,byref StrArray, Seperator,MaxSize)
'
'  Parse the string list seperated by 'Seperator' into the array
'  Returns the total number of elements parsed
'
'*****************************************************************************
function ParseString(StrSrc,byref StrArray, Seperator,MaxSize)

    SepLen = Len(Seperator)
    count = 0
    Loc = 1
    ParseStr = StrSrc
    MaxNdx = ubound(StrArray)

    '
    ' parse until the end of the list
    '
    while Loc > 0 and ParseStr <> "" and ParseStr <> Empty

         Loc = InStr( ParseStr, Seperator )

         if Loc > 0 then
            Str = left( ParseStr,Loc-1)
            ParseStr = Mid( ParseStr,Loc+SepLen)
						if Count < MaxNdx then
                Count = Count + 1
                StrArray(Count) = Str
            end if
        end if


    wend

    '
    ' Add in the last value
    '
    if ParseStr <> "" and ParseStr <> Empty then
				if Count < MaxNdx then
            Count = Count + 1
            StrArray(Count) = ParseStr
        end if

    end if
    ParseString = Count


end function
'*****************************************************************************
'
' Function AddUnits( Var, NumDecimals, Units )
'
'  Convert the variable var to a string with 'NumDecimals' decimal places and
'  add the string 'Units' to the end
'
'*****************************************************************************
function AddUnits( Var, NumDecimals, Units )
    Value = ToNumber( Var )
    if value <> empty then
        ValueStr = formatNumber(value,NumDecimals)
        AddUnits = ValueStr + " " + Units
    end if
end function

'*****************************************************************************
'
' Function ToNumber( VarValue )
'
'  Safe method to convert a string variable to a numeric variable that will
'  ignore any trailing text after the number
'  Example: ToNumber("33.45 Amps") will return 33.45
'
'*****************************************************************************
function ToNumber( VarValue )
    on error resume next
    ToNumber = Empty
    if not IsEmpty(VarValue) then
        TmpStr = CStr(VarValue)
        Length = len(VarValue)
        for i = 0  to Length
            TmpStr = Mid( TmpStr,1,Length -i)
            Result = CDbl( TmpStr )
            if Result <> empty then i = Length
        next
        ToNumber = Result
    end if
end function
'
'*****************************************************************************
' function CalcStat(Name,StartNdx,EndNdx)
'
' Calculates the min, max, average and standard devaition
' of a series of tags.  The results are placed in the
' variables <name>_min, <name>_max, <name>_avg, <name>_std
'
' It also returns the average
'
'  Example: CalcStat("current",1,10)
'
'*****************************************************************************

function CalcStat(Name,StartNdx,EndNdx)

    '
    ' Initialize the statistical variables
    '
    call SetVar(Name + "_min" , Min )
    call SetVar(Name + "_max" , Max )
    call SetVar(Name + "_avg" , Avg )
    call SetVar(Name + "_std" , Std )
    SumError2  = 0
    Count = 0

    '
    ' Calculate the average
    '
    for i = StartNdx to EndNdx
        Tmp = GetNdxVar(Name,i)
        Tmp = ToNumber( Tmp )
        if Tmp <> Empty then
            Min = Tmp
            Max = Tmp
            Count = Count + 1
            Sum = Sum + Tmp
        end if
    next

    '
    ' Calculate the avg, min, max and standard deviation
    '
    if Count > 0 then
        Avg = Sum / Count

        for i = StartNdx to EndNdx
            Tmp = GetNdxVar(Name,i)
            Tmp = ToNumber( Tmp )
            if Tmp <> Empty then
                if Tmp < Min then Min = Tmp
                if Tmp > Max then Max = Tmp
                Error = Avg - Tmp
                SumError2 = SumError2 + ( Error * Error )
            end if

        next

        Std = sqr( SumError2 / Count)

        call SetVar(Name + "_min" , Min )
        call SetVar(Name + "_max" , Max )
        call SetVar(Name + "_avg" , Avg )
        call SetVar(Name + "_std" , Std )


    end if

    CalcStat = Avg
end function
'
'*****************************************************************************
' SUB InsertRow(StartNdx,EndNdx,Var)
'
' Inserts a blank row into a table (note the data on the last row will be lost)
'
' StartNdx - Index of the row to insert
' EndNdx   - Index of the last row in the table
' Var      - Variable name to slide down (note this assumes no underscore
'            (use "name_" for names that have an underscore)
'
'  Example:  InsertRow(2,12,"Location_")
'
'*****************************************************************************
SUB InsertRow(StartNdx,EndNdx,Var)
    AtNdx = EndNdx
    for i = StartNdx to EndNdx
        Temp = GetNdxVarNu(Var,AtNdx-1)
        call SetNdxVarNu(Var,AtNdx,Temp)
        AtNdx = AtNdx - 1
    next

    call SetNdxVarNu(Var,StartNdx,Empty)
END SUB

'
'*****************************************************************************
' SUB DeleteRow(StartNdx,EndNdx,Var)
'
' Deletes a row in a table (note the data on the current row will be lost)
'
' StartNdx - Index of the row to delete
' EndNdx   - Index of the last row in the table
' Var      - Variable name to slide down (note this assumes no underscore
'            (use "name_" for names that have an underscore)
'
'  Example:  DeleteRow(2,12,"Location_")
'
'*****************************************************************************
SUB DeleteRow(StartNdx,EndNdx,Var)
    for i = StartNdx to EndNdx - 1
        AtNdx = i
        Temp = GetNdxVarNu(Var,AtNdx+1)
        call SetNdxVarNu(Var,AtNdx,Temp)
    next
    call SetNdxVarNu(Var,EndNdx,Empty)
END SUB
'
'*****************************************************************************
' FUNCTION TLang(StrIN)
'
' Translates StrIn into the target language
'
'
'  Example:  TLang("help")
'
'*****************************************************************************
FUNCTION TLang(StrIn)
   TLang = Application.TranslateStr(StrIn)
END FUNCTION

'*****************************************************************************
'
' FUNCTION GetTagName( Name )
'
' Returns the Tag Name
' NOTE: The parameter Name should ALWAYS be ".this!"
'
'*****************************************************************************

FUNCTION GetTagName( ThisTag )
    GetTagName  = ThisTag
    Length = len(ThisTag)
    if Length > 2 then
        if Mid(ThisTag,1,1) = "." and Mid(ThisTag,Length,1) = "!" then
            GetTagName   = Mid(ThisTag,2,Length-2)
        end if
    end if
END FUNCTION

'*****************************************************************************
'
' FUNCTION GetPrefix( Name )
'
' Gets the Prefix for this Tag
' NOTE: The parameter Name should ALWAYS be ".this!"
'
'*****************************************************************************
FUNCTION GetPrefix( ThisTag )
    GetPrefix = ""
    Length = len(ThisTag)
    if Length > 2 then
        if Mid(ThisTag,1,1) = "." and Mid(ThisTag,Length,1) = "!" then
            Pos = InStrRev(ThisTag,"__")
            if Pos > 2 then
                GetPrefix = Mid(ThisTag,2,Pos-2)
            end if
        end if
    end if
END FUNCTION

'*****************************************************************************
'
' FUNCTION MakeTagName( prefix, base )
'
' Returns the full tag name: prefix__base
'
'*****************************************************************************
FUNCTION MakeTagName( prefix, base )
  MakeTagName = base
  IF NOT IsBlankString(prefix) THEN
     MakeTagName = prefix + "__" + base
  END IF
END FUNCTION

'*****************************************************************************
'
' FUNCTION GetPrefixVar( prefix, tag )
'
' Returns value of variable 'tag' with given prefix
'
'*****************************************************************************
FUNCTION GetPrefixVar( prefix, tag )
   FullTag = MakeTagName( prefix, tag )
   GetPrefixVar = GetVar( FullTag )
END FUNCTION

'*****************************************************************************
'
' FUNCTION GetPrefixNdxVar( prefix, tag, ndx )
'
' Returns value of variable 'base' with given prefix and index
'
'*****************************************************************************
FUNCTION GetPrefixNdxVar( prefix, tag, ndx )
     FullTag = MakeTagName( prefix, tag )
    GetPrefixNdxVar = GetNdxVar( FullTag, ndx )
END FUNCTION

'*****************************************************************************
'
' SUB SetPrefixVar( prefix, tag, value )
'
' Sets value of variable 'tag' with given prefix
'
'*****************************************************************************
SUB SetPrefixVar( prefix, tag, value )
   FullTag = MakeTagName( prefix, tag )
   call SetVar( FullTag, value )
END SUB

'*****************************************************************************
'
' SUB SetPrefixNdxVar( prefix, tag, ndx, value)
'
' Sets the value of variable 'tag' with given prefix and index
'
'*****************************************************************************
SUB SetPrefixNdxVar( prefix, tag, ndx, value )
    FullTag = MakeTagName( prefix, tag )
    call SetNdxVar( FullTag, ndx, value )
END SUB

'*****************************************************************************
'
' SUB RunHome()
'
' Default method run when Home key is pressed. This should be overridden in
' forms if needed.
'
'*****************************************************************************
SUB RunHome()
END SUB


'*****************************************************************************
'
' FUNCTION CalcError(Actual,Desired)
'
' Calculate the percent error of a value
'
'*****************************************************************************
FUNCTION CalcError(x,y)
    CalcError = Empty
    if IsNumeric(x) and IsNumeric(y) and not IsEmpty(x) and Not IsEmpty(y) then
        ErrVal = abs(x-y) * 100
        CalcError = Divide( ErrVal , y )
    end if
END FUNCTION

'*****************************************************************************
'
' SUB RunTest()
'
' Default method run when Test key (F2) is pressed. This should be overridden
' in forms if needed.
'
'*****************************************************************************
SUB RunTest()
END SUB

'*****************************************************************************
'
' SUB SetStatus()
'
' Sets the status bar text
'
'*****************************************************************************
SUB SetStatus(Text)
   call Application.SetStatusText( Text )
END SUB


'*****************************************************************************
'
' FUNCTION RunTagCmd()
'
' Run a tag specific command
'
'*****************************************************************************
FUNCTION RunTagCmd(Tag,Cmd)
   RunTagCmd =  Application.RunTagCmd(Form.FormId,Tag,Cmd)
END FUNCTION


'*****************************************************************************
'
' SUB PostOnUpdate()
'
' This method is executed after all the controls onupdate and can be
' overridden
'
'*****************************************************************************
SUB PostOnUpdate2()
   DispId = Form.GetDispId("AutoDeficiency_1")
   if DispId > 0 then
     DefList = GetVar("AutoDeficiencyList")
     DefList2 = GetVar("DefList")

     if DefList <> Empty and DefList2 <> Empty then DefList = DefList & ","
     DefList =  DefList & DefList2

     If DefList <> Empty  Then
         DefList = DefList + TLang(" Failed")
     End If

     DefList3 = GetVar("DirectDeficiencyList")
     if DefList <> Empty  and DefList3 <> Empty then DefList = DefList & ","
     DefList =  DefList & DefList3

     DefList4 = GetVar("PdbAutoDefs")
     if DefList <> Empty  and DefList4 <> Empty then DefList = DefList & ","
     DefList =  DefList & DefList4

     if DefList <> Empty then call Application.RunTagCmd(Form.FormId, "AutoDeficiency_1","WrapTextToTable '" & DefList & "'" )
     if DefList =  Empty then call SetVar( "AutoDeficiency_1", Empty )
   else
       Call PostOnUpdate()
   end if
END SUB


SUB PostOnUpdate()
    DefList = GetVar("AutoDeficiencyList")
call alog("Postonupdate")
    if DefList <> Empty then call Applicaton.RunTagCmd("AutoDeficiency_1","SetWrappedText " + DefList + " Failed" )
END SUB


'*****************************************************************************
'
' SUB PreOnUpdate()
'
' This method is executed before all the controls onupdate and can be
' overridden
'
'*****************************************************************************
SUB PreOnUpdate()
    call SetVar("AutoDeficiencyList",Empty)
    call SetVar("DirectDeficiencyList",Empty)
END SUB

'*****************************************************************************
'
' SUB AddFailure()
'
' This method adds a failure note to the auto deficiency line
'
'*****************************************************************************
SUB AddFailure(DefNote)
    DefList = GetVar("AutoDeficiencyList")
    if DefList <> Empty then DefList = DefList + ","
    DefList = DefList + DefNote
    if Form.FormCurCalcDepth = 1 then Call SetVar( "AutoDeficiencyList",DefList)
END SUB


'*****************************************************************************
'
' SUB AddDeficiency()
'
' This method adds a deficiency note to the auto deficiency line.
' The note is added unchanged, and no "Failed" text is appended as is done
' by AddFailure().
'
'*****************************************************************************
SUB AddDeficiency(DefNote)
    DefList = GetVar("DirectDeficiencyList")
    if DefList <> Empty then DefList = DefList & ","
    DefList = DefList & DefNote
    MaxDepth = GetVar("FormMaxCalcDepth")
    if IsEmpty(MaxDepth) then MaxDepth = 1
    if Form.FormCurCalcDepth = MaxDepth then Call SetVar( "DirectDeficiencyList",DefList)
END SUB

'*****************************************************************************
'
' SUB ALog()
'
' This method adds a log message to the application log
'
'*****************************************************************************
SUB ALog(Msg)
    call Application.LogMsg(Msg)
END SUB


'*****************************************************************************
'
' SUB qcstr()
'
' This converts the data to a string but returns , back to .
'
'*****************************************************************************
function  qCStr(Num)
    str = CStr(Num)
    qcstr = replace(Str,",",".")
END function


'*****************************************************************************
'
' SUB DoExecuteCmdScriptlet()
'
' This function will run a script command ExecuteCmd
'
'*****************************************************************************
SUB DoExecuteCmd()
    str = GetVar("ExecuteCmd")
    call alog(  "Exec Cmd" & str )
    if str <> "" then  Execute str
END SUB

'
' *******************************************************
' Calculate a log base 10
' *******************************************************
'
Function Log10(Value)
    Log10 = Log(Value) / Log(10)
End Function
' *******************************************************
' Retrieves Test Status Name from DB
'*******************************************************
Function GetTestStatus(ResultGUID)
With Form
  TestStatusGUID = GetColumnDataFromResult("TestStatusGUID", ResultGUID)
  GetTestStatus = GetColumnDataFromTestStatus("Name", TestStatusGUID)
  call SetVar("PdbTestStatusDatabaseGUID", TestStatusGUID)
  call SetVar("PdbTestStatusDatabaseName", GetTestStatus)
  call SetVar("PdbTestStatusDatabasePF", GetPassFail)
end With
end Function

'*********************************************
' Retrives inputted column data from Result Header table
'*********************************************
Function GetColumnDataFromResult(ColumnName, ResultGUID)
With Form
  bMultiSelect = 0
  call Application.querydatabase(.FormId, "SELECT " & ColumnName & " from Results_Header WHERE ResultsGUID = '" & ResultGUID & "'", bMultiSelect)
  GetColumnDataFromResult = GetVar("db_" & ColumnName)
end With
end Function

'*********************************************
'Retrives inputted column data from Test Status table
'*********************************************
Function GetColumnDataFromTestStatus(ColumnName, TestStatusGUID)
with Form
  bMultiSelect = 0
  call Application.querydatabase(.FormId, "SELECT " & ColumnName & " from PdbTestStatus WHERE TestStatusGUID = '" & TestStatusGUID & "'", bMultiSelect)
  GetColumnDataFromTestStatus = GetVar("db_" & ColumnName)
end With
end Function

'*********************************************
' Push changes to Result Header table
'*********************************************
Function UpdateColumnDataInResult(ColumnName, ResultGUID, DataToSet)
with Form
  bMultiSelect = 0
  call Application.querydatabase(.FormId, "UPDATE Results_Header SET "  & ColumnName & " = '" & DataToSet & "' WHERE ResultsGUID = '" & ResultGUID & "'", bMultiSelect)
end With
end Function

'*********************************************
' Get list of Test Statuses from Test Status table
'*********************************************
Function GetTestStatusOptions()
with Form
  bMultiSelect = 1
  call Application.querydatabase(.FormId, "SELECT Name from PdbTestStatus WHERE bIsDel <> 1 AND TestStatusGUID <> '@undefined@status' ORDER BY Name ASC, ePassFail ASC", bMultiSelect)
  AppendStr = GetVar("db_Name_1")
  NumRecords = GetVar("db_NumRecords")
  For ndx = 2 to NumRecords
    AppendStr = AppendStr & ", " & GetNdxVar("db_Name", ndx)
  Next
  AppendStr = AppendStr & ", " & "Cancel"
  GetTestStatusOptions = AppendStr
end With
end Function

'*********************************************
' Test Status picker setup
'*********************************************
Function DisplayPickerAndRetVal(ChoiceList)
  call Application.DisplayPicker(Form.FormId, 0, 0, 0, "ChoicePicker", ChoiceList)
  ChoiceNum = GetVar("ChoicePicker")
  ChoiceArray = Split(ChoiceList, ", ")
  ChoiceNdx = 1
  for each Choice in ChoiceArray
    if ChoiceNum = ChoiceNdx then
      RetVal = Choice
      exit for
    end if
    ChoiceNdx = ChoiceNdx + 1
  next
  DisplayPickerAndRetVal = RetVal
end Function

'*********************************
' Fill Test Status picker with list of Test Statuses
'*********************************
Function InitTestEnum()
  TestStatusEnum = GetTestStatusOptions()
  call InitEnumeration(TestStatusEnum)
end Function

'***************************************** **************
' Test Status picker selection
'*******************************************************
Function SetTestType(TestType, ResultGUID)
With Form
  bMultiSelect = 1
  call Application.querydatabase(.FormId, "SELECT Name, TestStatusGUID from PdbTestStatus WHERE bIsDel <> 1 AND TestStatusGUID <> '@undefined@status' ORDER BY Name ASC, ePassFail ASC", bMultiSelect)
  DataToSet = GetNdxVar("db_Name", TestType)
  call SetVar("GUIDToSet", GetNdxVar("db_TestStatusGUID", TestType))
  call SetVar("TestStatus", DataToSet)
  call SetVar("TestStatusToSet", DataToSet)
  SetTestType = DataToSet
end With
end Function

'*********************************************
'Retrieves Database Schema
'*********************************************
Function GetSchemaFromDB
With Form
  bMultiSelect = 0
  call Application.querydatabase(.FormId, "SELECT ConfigValue FROM PdbConfig_Local WHERE Name = 'PowerDB Schema'", bMultiSelect)
  GetSchemaFromDB = GetVar("db_ConfigValue")
end With
end Function

'*******************************************************
' On Select of Test Status
'*******************************************************
Function ShowandGetTestStatus()
With Form
  ResultGUID = GetVar("PdbResultsGUID")
  if ResultGUID = Empty then
    msgbox("Results must be saved before altering Test Status.")
  else
    call DisplayPickerAndRetVal(GetTestStatusOptions())
    call SetTestType(GetVar("ChoicePicker"), ResultGUID)
    if DataToSet <> 1 then
      call UpdateColumnDataInResult("TestStatusGUID", ResultGUID, GetVar("GUIDToSet"))
    end if
    ShowandGetTestStatus = GetTestStatus(ResultGUID)
    DBSchema = GetSchemaFromDB()
    call SetVar("PdbTestStatusDatabaseSchema", DBSchema)
  end if
end With
end Function
'*******************************************************
' Get Scripting WildCard to be used in database
'*******************************************************
Function GetPdbWildCardForDB()
With Form
  GetPdbWildCardForDB = "{PDB_DB_WILDCARD}"
end With
end Function
'*******************************************************
' Loads test status from results header
'*******************************************************
Function LoadTestStatus()
With Form
  bMultiSelect = 0
  TestStatusGUID = GetVar("PdbTestStatusGUID")
  call Application.querydatabase(.FormId, "SELECT Name from PdbTestStatus WHERE TestStatusGUID ='" & TestStatusGUID &"'", bMultiSelect)
  LoadTestStatus = GetVar("db_Name")
end With
end Function



'
'
Sub DoOnInitialUpdateModel
'

	With Form

tagstohide="describe,describelbl,describeErr,test_date,test_datelbl,test_dateErr,service_date,service_datelbl,service_dateErr,sleeves_inspdate,sleeves_inspdatelbl,sleeves_inspdateErr,incident_enrglbl,incident_enrgErr,incident_enrg,hppelbl,hppe,hppeErr"

call hidetags(tagstohide)

.fdate=.FormOpenTime
.sname=.PdbTesterName
.ticketsd=.FormOpenTime
.customer=.PdbCustomerName

strlen = Instr(GetVar("PdbEqpLvlTwo"),"-")
        if (strlen = 0) then 
            call SetVar("SiteID", GetVar("PdbEqpLvlTwo"))
        else
            call SetVar("SiteID", Left(GetVar("PdbEqpLvlTwo"),strlen - 1))
        end if
SiteID = GetVar("SiteID")
call LoadSiteAddress(SiteID)

if Len(GetVar("ticketno")) = 0 or IsChangedEx("PdbJobNumber", Key) then 
    Ticket = .PdbJobNumber
    DashPos = InStr(Ticket, "-")
      
    If DashPos > 0 Then
       Ticket = Left(Ticket, DashPos - 1)
       
    End If

    Call SetVar("ticketno", Ticket)
end if

call Application.Zoom100(form.FormID)
	End With

End Sub

'
'
Sub DoOnUpdateModel
'

	With Form

.completedby=.PdbTesterName
if GetVar("conf1")=1 then
  Call RunTagCmd("conf1", "bgcolor " & CLR_WHITE)
end if
if GetVar("conf2")=1 then
   Call RunTagCmd("conf2", "bgcolor " & CLR_WHITE)
end if
if GetVar("conf3")=1 then
   Call RunTagCmd("conf3", "bgcolor " & CLR_WHITE)
end if
if GetVar("conf4")=1 then
   Call RunTagCmd("conf4", "bgcolor " & CLR_WHITE)
end if
	End With

End Sub

'
'Functions 
'
function hidetags(tagarray)
   tagsArray = Split(tagarray, ",")
        Ndx = 1
        for each tagname in tagsArray
           call SetVisible(tagname, false)
           Ndx = Ndx + 1
        next
end function

function readonlytags(tagarray)
   tagsArray = Split(tagarray, ",")
        Ndx = 1
        for each tagname in tagsArray
           tagname_readonly=true
           Ndx = Ndx + 1
        next
end function
    

function showhide(var, tagname, tagnamelbl)
    if var=1 then
        call SetVisible(tagname, true)
        call SetVisible(tagnamelbl, true)
        
        call RunTagCmd(tagname,"AddReqColorBox 1")
        'call ValidateIsNotBlankField(tagname)
                
       else
        call SetVisible(tagname, false)
        call SetVisible(tagnamelbl, false)
      end if
end function

function setstyle(tagname)
    value=GetVar(tagname)
    if value="0" then
      Call RunTagCmd(tagname, "bgcolor " & CLR_RED)
      else if IsBlankString(Value) then
       call RunTagCmd(tagname,"AddReqColorBox 1")
       end if
    end if
end function

function requiredtags(tagarray)
        requiredarray=""
        tagsArray = Split(tagarray, ",")
        Ndx = 1
        for each tagname in tagsArray
          Value = GetVar(tagname)
          Value = Trim(Value)
        if Value="0" OR IsBlankString(Value) then
         requiredarray=requiredarray & tagname & ", " 
         call setstyle(tagname)
        End If
        Ndx = Ndx + 1
        next
        
        if Not requiredarray="" then
            call Application.SetStatusText("Warning: missing values!")
            'MsgBox("Missing values: " & requiredarray)
            call Application.ChangePage(form.FormId, 3, 0)
            requiredtags=false
        else requiredtags=true
        end if
end function

    
FUNCTION LoadSite(JobNumber)
    SQL = "SELECT Info2 FROM PdbJob WHERE [JobNumber] = '" & JobNumber & "'"
    Call Application.QueryDatabase(Form.formId, SQL, 0)
    jobumber=GetVar("db_Info2")
    call SetVar("SiteID", jobnumber)
    end function

 
FUNCTION LoadSiteAddress(SiteID)
    SQL = "SELECT TOP 1 C.[Name], C.CompanyGUID, A.AddrGuid, A.AddrLn1, A.AddrLn2, A.City, A.State, A.Country, A.Zip, A.FirstName, A.LastName " &_
          " FROM PdbAddrInfo A LEFT JOIN PdbAddrHeader C on A.CompanyGuid = C.CompanyGUID " &_
          " WHERE A.[Type] = 2 And A.[bIsDel] = 0 And (A.AddrNeutralId = '" & SiteID & "' OR A.AddrNeutralId = '" & SiteID & "-2')"
    
    Call SetVar("db_AddrGuid", Empty)      
    
    Call Application.QueryDatabase(Form.formId, SQL, 0)
    
    AddrGuid = GetVar("db_AddrGuid")
    If Not IsBlankString(AddrGuid) Then
        Company = GetVar("db_Name")
        CompanyGuid = GetVar("db_CompanyGuid")
        AddrLn1 = GetVar("db_AddrLn1")
        AddrLn2 = GetVar("db_AddrLn2")
        City = GetVar("db_City")
        State = GetVar("db_State")
        Country = GetVar("db_Country")
        Zip = GetVar("db_Zip")
        FirstName = GetVar("db_FirstName")
        LastName = GetVar("db_LastName")
        Call SetVar("address", AddrLn1 & ", " & AddrLn2 & ", " & City & ", " & Country & ", " & State & ", " & Zip)

       
        'call aLog("Found Site Address: " & AddrGuid & ", " & CompanyGuid & ", " & Company & ", " & AddrLn1 & ", " & AddrLn2 & ", " & City & ", " & State & ", " & Zip & ", " & FirstName & ", " & LastName)
    Else
        'call aLog("Did not find site address")
    End If    
    
    
End FUNCTION

Sub CreateAfile(TextFileContents, CreateFilePath)
   Dim fso, MyFile
   Set fso = CreateObject("Scripting.FileSystemObject")
   Set MyFile = fso.CreateTextFile(CreateFilePath, True)
   MyFile.WriteLine (TextFileContents)
   MyFile.Close
End Sub
'''''email


Function GetFileDlg(sIniDir,sFilter,sTitle)
 GetFileDlg=CreateObject("WScript.Shell").Exec("mshta.exe ""about:<object id=d classid=clsid:3050f4e1-98b5-11cf-bb82-00aa00bdce0b></object><script>moveTo(0,-9999);function window.onload(){var p=/[^\0]*/;new ActiveXObject('Scripting.FileSystemObject').GetStandardStream(1).Write(p.exec(d.object.openfiledlg('" & sIniDir & "',null,'" & sFilter & "','" & sTitle & "')));close();}</script><hta:application showintaskbar=no />""").StdOut.ReadAll
End Function
Function FolderDlg()
    Const MY_COMPUTER = &H11&
    Const WINDOW_HANDLE = 0
    Const OPTIONS = 0
    
    Set objShell = CreateObject("Shell.Application")
    Set objFolder = objShell.Namespace(MY_COMPUTER)
    Set objFolderItem = objFolder.Self
    strPath = objFolderItem.Path
    
    Set objShell = CreateObject("Shell.Application")
    Set objFolder = objShell.BrowseForFolder _
        (WINDOW_HANDLE, "Select a folder:", OPTIONS, strPath)
          
    If objFolder Is Nothing Then
        FolderDlg = Empty
    Else
        Set objFolderItem = objFolder.Self
        objPath = objFolderItem.Path
        
        '
        ' return the folder path
        '
        FolderDlg = objPath
    End If
End Function


sub OnFileEndSaveToPDF3(var)
with form
    if var=0 then
    ticket=GetVar("ticketno")
    ticketdate=GetVar("ticketsd")
    customer=GetVar("customer")
    address=GetVar("address")
    
    emails=GetVar("email_list")
    EmailAddressString = emails
    
    
    SubHeaderStr = "Site EHS Survey"

    BodyStr =  "Ticket: " & ticket & vbCrLf & "Ticket Start Date: " & ticketdate & vbCrLf & "Customer: " & customer & vbCrLf & "Address:" & address
                      	
	' Open Outlook and create the email
	'
    Set ol = CreateObject("Outlook.Application")
    Set ns = ol.getNamespace("MAPI")
    Set newMail = ol.CreateItem(olMailItem)
    newMail.Subject = SubHeaderStr
    newMail.Body = BodyStr
    
    '
    ' EmailAddressString must contain a value otherwise a scripting 
    ' error will occur when you try to run newMail.Recipients.Add()
    '
    if Len(EmailAddressString) > 0 then
        newMail.Recipients.Add(EmailAddressString)
    end if
    
   'newMail.Attachments.Add("C:\ProgramData\PowerDB\TmpFile.pdf").DisplayName = "TmpFile.pdf"
    
    newMail.Display
    
   end if
end with
end sub

'****************************************************************************
' Function GetVarForSQL()
'
' Return the value for the given tag name formatted for use in a SQL Select.
' i.e. If it's a string value, it is enclosed in single quotes.
'
'****************************************************************************
Function GetVarForSQL( TagName )        

    Value = GetVar( TagName )
    GetVarForSQL = PrepareSQLVar( Value )

End Function


'****************************************************************************
' Function PrepareSQLVar()
'
' Return the given value formatted for use in a SQL Select.
' i.e. If it's a string value, it is enclosed in single quotes.
' any quotes are escaped
'
'****************************************************************************
Function PrepareSQLVar( Value )

    If IsEmpty( Value ) Or IsNull( Value ) Then
	PrepareSQLVar = Empty
    ElseIf VarType( Value ) = vbString Then
        Value = Replace( Value, "'", "''" )
        PrepareSQLVar = "'" & Value & "'"
    Else
        PrepareSQLVar = Value
    End If

End Function


'
'#####################################################
'EMBEDDED CONTROL User Data
'#####################################################
'

'
'
Sub DoOnUpdate_User_Data___User_Data_t0_l0_r40_b40
'

	With Form


if GetVar("PdbDeviceGuid")  = "zk00037r@\]d2/?" then 
    if GetVar("Manufacturer_List") <> Empty and GetVar("Mfr") = Empty then 
        call SetVar("Mfr", GetVar("Manufacturer_List") )
        call SetVar("Manufacturer_List",Empty)
        call SetVar("SerNo", GetVar("SerialNo") )
        call SetVar("VA_1", GetVar("kva1") )
        call SetVar("VA_2", GetVar("kva2") )
        call SetVar("COOLANT", GetVar("COOLANT") )
        call SetVar("Phases", GetVar("Phase") )
        call SetVar("TankType", GetVar("TYPE") )
        call SetVar("Type", GetVar("CLASS") )
        call SetVar("WdgMaterial", GetVar("windmaterial") )
        call SetVar("Weight",Empty)
        call SetVar("Bil", CStr(GetVar("KVPri") ) )
        call SetVar("Weight", GetVar("TotalWeight") )
        call SetVar("Gallons", GetVar("maintankcap") )

        '
        ' ERS to provide mapping
        '
        if .TYPE = "Sealed Gas Filled Dry Type" then .TankType = "Sealed"




    end if 
end if 
	End With

End Sub

'
'Functions 
'
function MakeSQLSafe(text)
    MakeSQLSafe=Replace(text,"'","''")
end Function


'
'#####################################################
'EMBEDDED CONTROL Optima Data - Do Not Change
'#####################################################
'

'
'
Sub DoOnInitialUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16
'

	With Form

version = getVersion
call SetVar("PdbVersion", version)
.DeclareVar("PdbDefaultMetric")
.DeclareVar("UseMetric")
.DeclareVar("LastUseMetric")
.DeclareVar("foot_meter_str")
.DeclareVar("inch_cm_str")
.DeclareVar("lc_foot_meter_str")
.DeclareVar("lc_inch_cm_str")
.DeclareVar("bNarrowFormat")
call SetTagIfEmpty("DefaultFreq",60)
on error resume next 
call Application.AddSavedVar(Form.FormId,"UseMetric")
call Application.AddSavedVar(Form.FormId,"AssetID_SN")
on error goto 0
.DeclareVar("TemperatureC")
 
	End With

End Sub

'
'
Sub DoOnUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16
'

	With Form

'
' Setup metric / imperial variables
'
if IsEmpty( .UseMetric)  then .UseMetric = .PdbDefaultMetric
if .UseMetric <> .LastUseMetric  or IsEmpty( .LastUseMetric ) then 
    .LastUseMetric = .UseMetric
    if .UseMetric then 
        tmp_inch_cm_str = "cm"
        tmp_foot_meter_str = "METERS" 
    else 
        tmp_inch_cm_str = "INCHES"
        tmp_foot_meter_str = "FEET"
    end if 
    .foot_meter_str =  Translate( tmp_foot_meter_str ) 
    .inch_cm_str =  Translate( tmp_inch_cm_str)
    .lc_foot_meter_str =  Translate( LCase( tmp_foot_meter_str ) ) 
    .lc_inch_cm_str =  Translate(LCase( tmp_inch_cm_str))
    
end if 




	End With

End Sub

'
'Functions 
'
'
' ***************************************************
' UpdateGraphOptions()
' ***************************************************
'Backwards compatibility, Prefix param was never used.
sub UpdateGraphOptions(Prefix, Chart, NumTraces)
    call UpdateGraphOptionsEx(Chart, NumTraces)
end sub
sub UpdateGraphOptionsEx(Chart, NumTraces)
    if GetVar(Chart&"_Init") = 1 then 
        if RunTagCmd(Chart,"IsChart") = "ValidChart" then
            call SetVar(Chart&"_Init", 0)         
            if( GetPrefixVar(Chart, "bAutoScaleX") = 0 or GetPrefixVar(Chart, "bAutoScaleY") = 0 ) then
                for Ndx = 1 to NumTraces
                    GraphMinXVal = GetPrefixVar(Chart,"GraphMinXVal")
                    GraphMaxXVal = GetPrefixVar(Chart,"GraphMaxXVal")
                    GraphMinYVal = GetPrefixVar(Chart,"GraphMinYVal")
                    GraphMaxYVal = GetPrefixVar(Chart,"GraphMaxYVal")
                    if GetPrefixVar(Chart,"bXAxisIsTime") then
                        GraphMinXVal = GraphMinXVal / 60
                        GraphMaxXVal = GraphMaxXVal / 60
                    end if

                    if GetPrefixVar(Chart,"bYAxisIsTime") then
                        GraphMinYVal = GraphMinYVal / 60
                        GraphMaxYVal = GraphMaxYVal / 60
                    end if
                
                    CmdStr = "SetScale " & Ndx & "," & CStr(GraphMinXVal) & "," & CStr(GraphMinYVal) & "," & CStr(GraphMaxXVal) & "," & CStr(GraphMaxYVal)
                    call RunTagCmd(Chart, CmdStr)
                next
            end if
        
            for Ndx = 1 to NumTraces
                if GetPrefixVar(Chart,"bHideGraphSymbols") = 1 then
                    call RunTagCmd(Chart,"ShowTraceSymbol "&Ndx&",0")
                else
                    call RunTagCmd(Chart,"ShowTraceSymbol "&Ndx&",1")                        
                end if
            next
            
            call RunTagCmd(Chart, "Zoom100" )
            CmdStr = "SetYAutoScale " & GetPrefixVar(Chart, "bAutoScaleY")
            call RunTagCmd(Chart, CmdStr)

            CmdStr = "SetXAutoScale " & GetPrefixVar(Chart, "bAutoScaleX")
            call RunTagCmd(Chart, CmdStr)            
        end if
    end if
end sub
'
' ***************************************************
' UpdateGraphOptionsExtraYAxis()
' ***************************************************
sub UpdateGraphOptionsExtraYAxis(Chart)
    if GetVar(Chart&"_Init") = 1 then 
        
        if RunTagCmd(Chart,"IsChart") = "ValidChart" then        
            call SetVar(Chart&"_Init", 0)
            if( GetPrefixVar(Chart, "bAutoScaleX") = 0 or GetPrefixVar(Chart, "bAutoScaleY") = 0 ) then
                GraphMinXVal = GetPrefixVar(Chart,"GraphMinXVal")
                GraphMaxXVal = GetPrefixVar(Chart,"GraphMaxXVal")
                GraphMinYValLeft = GetPrefixVar(Chart,"GraphMinYValLeft")
                GraphMaxYValLeft = GetPrefixVar(Chart,"GraphMaxYValLeft")
                GraphMinYValRight = GetPrefixVar(Chart,"GraphMinYValRight")
                GraphMaxYValRight = GetPrefixVar(Chart,"GraphMaxYValRight")

                if GetPrefixVar(Chart,"bXAxisIsTime") then
                    GraphMinXVal = GraphMinXVal / 60
                    GraphMaxXVal = GraphMaxXVal / 60
                end if

                if GetPrefixVar(Chart,"bYAxisIsTimeLeft") then
                    GraphMinYValLeft = GraphMinYValLeft / 60
                    GraphMaxYValLeft = GraphMaxYValLeft / 60
                end if

                if GetPrefixVar(Chart,"bYAxisIsTimeRight") then
                    GraphMinYValRight = GraphMinYValRight / 60
                    GraphMaxYValRight = GraphMaxYValRight / 60
                end if
            
                CmdStr = "SetScale 1," & CStr(GraphMinXVal) & "," & CStr(GraphMinYValLeft) & "," & CStr(GraphMaxXVal) & "," & CStr(GraphMaxYValLeft)
                call RunTagCmd(Chart, CmdStr)

                CmdStr = "SetScale 2," & CStr(GraphMinXVal) & "," & CStr(GraphMinYValRight) & "," & CStr(GraphMaxXVal) & "," & CStr(GraphMaxYValRight)
                call RunTagCmd(Chart, CmdStr)                
            end if
        
            if GetPrefixVar(Chart,"bHideGraphSymbols") = 1 then
                call RunTagCmd(Chart,"ShowTraceSymbol 1,0")
            else
                call RunTagCmd(Chart,"ShowTraceSymbol 1,1")                        
            end if
            
            call RunTagCmd(Chart, "Zoom100" )
            CmdStr = "SetYAutoScale " & GetPrefixVar(Chart, "bAutoScaleY")
            call RunTagCmd(Chart, CmdStr)

            CmdStr = "SetXAutoScale " & GetPrefixVar(Chart, "bAutoScaleX")
            call RunTagCmd(Chart, CmdStr)            
        end if
    end if
end sub
'
' ***************************************************
' SetTagIfEmpty()
' ***************************************************
sub SetTagIfEmpty(Tag,Value)
    val = Getvar(Tag)
    if isEmpty(Val) then call SetVar(Tag, Value )
end sub
'
' ***************************************************
' UpdatePrefixCurve()
' ***************************************************
sub UpdatePrefixCurve(Prefix)
    Prefix = TrimUnderscores(Prefix)
    

    if IsChanged( Prefix  & "__Type") then Changed = true
    if IsChanged( Prefix  & "__Manufact") then Changed = true
    if IsChanged( Prefix  & "__Model") then Changed = true
    if IsChanged( Prefix  & "__Curve") then Changed = true
    if IsChanged( Prefix  & "__Tdm")   then Changed = true
    call alog("Update Prefix Curve " & Prefix & " Changed = " & Changed )

    if Changed then 
        TypeStr = GetVar( Prefix & "__Type")
        Manufact = GetVar( Prefix & "__Manufact")
        Model = GetVar( Prefix & "__Model")
        Name = GetVar( Prefix & "__Curve")
        k = GetVar(Prefix & "__Tdm" )
call alog("curve type = " & TypeStr )

        call  LoadCurveFromTable( Prefix , TypeStr, Manufact, Model,  Name , k )
    end if 
end sub 

'
' ***************************************************
' TrimUnderscores()
' ***************************************************
'
function TrimUnderscores(Txt)
    Length = Len(Txt)
    if Length and right(Txt,1) = "_" then Txt = left(Txt,Length-1)
        
    Length = Len(Txt)
    if Length and right(Txt,1) = "_" then Txt = left(Txt,Length-1)
    TrimUnderscores = Txt
end function 

'
' *******************************************************
' Load a curve from the curve library
' Fills X_#, Y_# variables 
' Also sets NumCurvePoints variable
' *******************************************************
'
sub LoadCurveFromTable( Prefix , TypeStr, Manufact, Model,  Name , k )
    Rev = GetInterfaceRev()
    call aLog("Rev = " & Rev)
    If Rev < 17 Then
        call LoadCurveFromTablePreRev17(Prefix, TypeStr, Manufact, Model,  Name , k)
    Else
        XScaleMult = GetVar("xScaleMult")
        If IsBlankString(XScaleMult) Then
            XScaleMult = 1
        End If
        
        '
        ' Set the equation to blank - See bugzilla 6040 for details
        '
        call SetVar(Prefix & "__Equation", "" )
        
        call Application.LoadCurve(Form.FormId, TypeStr, Manufact, Model, Name, k, Prefix, XScaleMult, "")        
    End If
    
end sub


'
' *******************************************************
' Old Version of LoadCurveFromTable, only called if Automatin interface revision is < 17
'
' Load a curve from the curve library
' Fills X_#, Y_# variables 
' Also sets NumCurvePoints variable
' *******************************************************
'
sub LoadCurveFromTablePreRev17( Prefix , TypeStr, Manufact, Model,  Name , k )
    
    xScaleMult = 1
    Length = len(Prefix)
    if Length > 2 and  Right( Prefix,2) = "__" then Prefix = Left(Prefix,length-2)
    
    with form
'    SelectStr = "SELECT XData,YData,bUseEquation,StartX,EndX,ScaleType, Equation, ErrorPlus, ErrorMinus, ErrorAbsPlus,ErrorAbsMinus from Curves where " _
'                 &  " CurveName = '" & Name  &  "' AND CurveType = '" & TypeStr & "' AND Manufacturer = '" & Manufact & "'" & " AND Model = '" & Model & "'"
    SelectStr = "SELECT * from Curves where " _
                 &  " CurveName = '" & Name  &  "' AND CurveType = '" & TypeStr & "' AND Manufacturer = '" & Manufact & "'" & " AND Model = '" & Model & "'"

    call SetVar(Prefix & "__NumCurvePoints",0)
                 
   call alog("tdm= " & k & "   select = " & SelectStr )
    call SetVar("db_ErrorMsg",Empty)
    call Application.QueryDatabase(Form.FormId, SelectStr,0)
    
    if .SqlError = Empty then 

        Call SetVar( Prefix & "__ErrorPlus",.db_ErrorPlus)
        Call SetVar( Prefix & "__ErrorMinus",.db_ErrorMinus)
        Call SetVar( Prefix & "__ErrorAbsPlus",.db_ErrorAbsPlus)
        Call SetVar( Prefix & "__ErrorAbsMinus",.db_ErrorAbsMinus)
        Call SetVar( Prefix & "__Equation",.db_Equation)
        Call SetVar( Prefix & "__ResetEquation", GetVar("db_ResetEquation") )
        
        ' call alog("Setting equation '" &  Prefix & "__Equation' = " & .db_Equation )
        
        if .db_bUseEquation then 
            StLogLog = 4
            StSemiLogX = 2
            ScaleType = ToNumber( .db_ScaleType )
            StartX = .db_StartX
            if StartX = 1 then StartX = 1.01
            '
            ' If log-log or semi-log X 
            ' 
            if ScaleType = StLogLog   or ScaleType = StSemiLogX  then 
                call CalcCurveLogX(Prefix,StartX, .db_EndX, 100, k, " y = "  & .db_Equation )
            else 
                call CalcCurveLinear(Prefix,StartX, .db_EndX, 100, k, " y = "  & .db_Equation )
            end if 
            
        else 
            if .db_Equation = "abs" then xScaleMult = GetVar("xScaleMult")
        
            for i = 1 to 100 
                call SetNdxVar(Prefix & "__x_" & Ndx,i, Empty )
                call SetNdxVar(Prefix & "__y_" & Ndx,i, Empty )
            next
        
            if  .db_ErrorMsg <> "" then 
                call alog("DB ERROR: " & .db_ErrorMsg )
            elseif .db_NumRecords > 0 then 
                call alog("xdata = " & .db_xData )
                call alog("ydata = " & .db_yData )
                dim XData(200)
                dim YData(200)
                call ParseString( .db_XData, XData,"|",200)
                call ParseString( .db_YData, YData,"|",200)
                
                                
                '
                ' xScale Mult
                ' 
                if xScaleeMult = Empty then xScaleMult = 1
                '
                ' Check for time dial data
                ' 
                bTimeDialData = false 
                for i = 2 to 200 

                    if IsEmpty( XData(i) ) then exit for

                    xDataN = toNumber(XData(i))
                    xDataNMinus1 = ToNumber(xdata(i-1))


                   if xDataN < xDataNMinus1 then 
                       bTimeDialData = true
                       exit for 
                   end if 
                next 
                call alog("TimeDialData = " & bTimeDialData )
                Tdm = ToNumber( k )
                if bTimeDialData and Tdm <> Empty and Tdm >= .5 and Tdm < 12 then 
                    TdData = array( .5,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17)
                    TdNdx = 0
                    dim YTd(20,100)
                    PtNdx = 1
                    for i = 1 to 200 

                        xDataN = toNumber(XData(i))
                        xDataNMinus1 = ToNumber(xdata(i-1))

                     '   call alog("XData(" & i & " ) = " & xDataN & "," & xDataNMinus1 )
                        if IsEmpty( XData(i) ) then exit for
                        
                       if i > 1 and  xDataN < xDataNMinus1 then 
                           TdNdx = TdNdx + 1 
                           PtNdx = 1
                      '     call alog("New Time Dial" & xData(i) & "," & xData(i-1) )'

                        end if 
                        YTd(TdNdx, PtNdx ) = yData(i)
                        ptNdx = PtNdx + 1
                        call alog( "Ydata(" & TdNdx & "," & PtNdx & " ) = " & yData(i))

                    next 
                    
                    for i = 0 to 12 
                       call alog("Tdm = " & Tdm & "   TdData(" & i & ") = " & TdData(i))
                       if Tdm <= TdData(i) then 
                          Ndx1 = i - 1
                          Ndx2 = i 
                          if Tdm = TdData(i) then Ndx1 = i
                          exit for 
                      end if 
                    next
                    Tdm1 = TdData( Ndx1)
                    Tdm2 = TdData( Ndx2)
                    PercentOf2 = Tdm - Tdm1 
                    if PercentOf2 then PercentOf2 = Divide( PercentOf2 , (Tdm2 - Tdm1))
                    call alog("Pof2 = " & PercentOf2 & "   " & Ndx1 & "," & Ndx2 & "    Tdm12 = " & Tdm1 & "," & Tdm2)
                    
                    for i = 1 to 20 
                      Y = Ytd( Tdm1,i)
                      if IsEmpty(Y) then 
                          YData(i) = Y
                          XData(i) = Empty 
                          exit for 
                      end if 
                      if PercentOf2 then Y = Y + (( Ytd(Tdm2,i) - Y ) * PercentOf2 )
                      call alog( "Y(" & i & ") = " & Y )
                       yData(i) = Y
                    next 
                end if 
                bdone = false
                for i = 1 to 200 
                    if IsEmpty( XData(i) ) then  bDone = true 
                   if bDone then 
                        call SetNdxVar(Prefix & "__x" & Ndx,i, Empty)
                        call SetNdxVar(Prefix & "__y" & Ndx,i, Empty)
                    else
        
                        call SetNdxVar(Prefix & "__x" & Ndx,i, XData(i) * xScaleMult  )
                        call SetNdxVar(Prefix & "__y" & Ndx,i, yData(i) )
                        call alog( i & "," & xData(i) & "," & yData(i) )
                        call SetVar(Prefix & "__NumCurvePoints",i)
                    end if 
                next
            end if 
        end if
    end if 
    end with 
end sub 

'
' *******************************************************
' Evaluate a point on a curve that is already loaded 
'
' Kept for backwards compatibility, calls EvalCurveExtraParams()
'
' *******************************************************
'
function EvalCurve( Curve, Mult, Tdm )
    
    EvalCurve = EvalCurveExtraParams( Curve, Mult, Tdm, "" )
    
end function

'
' *******************************************************
' Evaluate a point on a curve that is already loaded 
' *******************************************************
'
function EvalCurveExtraParams( Curve, Mult, Tdm, ExtraParams )
    Equation = GetVar( Curve & "Equation")
    
    ' set defaults for pickup/tap
    pickup = 1
    tap = 1

    'override with ExtraParams
    if Not IsBlankString(ExtraParams) then
       on error resume next
       CommandList = Replace(ExtraParams, ",", ":")
       call Execute(CommandList)
       on error goto 0 
    end if

    k = Tdm
    
     
    x = Mult
    M = Mult
    I = Mult
    y = Empty
    on error resume next
    call Execute( " y = " & Equation )
    on error goto 0 

 call alog("EvalCurve( " & Curve & "," & M & "," & Tdm & ") = " & y & "      Eq = " & Equation )
    '
    ' If no equation, try a chart lookup
    '
    if IsEmpty(y) then 
        Chart = GetVar(Curve & "Chart")
        Trace = GetVar(Curve & "Trace")
        call alog("EvalCurve("&Curve&") " & Chart & "," & Trace )
        if Chart <> Empty and Trace <> Empty then y =  RunTagCmd(Chart,"LookupY " & Trace & " " & Mult )
    end if 

    EvalCurveExtraParams =  y 
end function 


' *********************************************************
' Sweep through a user defined equation with a log x scale
'
' Kept for backwards compatibility, calls CalcCurveLogXExtraParams()
'
' *********************************************************
'
sub CalcCurveLogX(Prefix,StartVal,EndVal,NumPoints,k,Equation)
    
    Call CalcCurveLogXExtraParams(Prefix,StartVal,EndVal,NumPoints,k,Equation,"")
    
end sub
    
'
' *******************************************************
' Sweep through a user defined equation with a log x scale 
'
' ExtraParams: Values of extra parameters to be made available to
'              equation. Should be colon (or comma) separated
'
'              eg: "pickup=4.3,xx=12".
'
'              Note: Only pickup and tap are supported for now
' *******************************************************
'
sub CalcCurveLogXExtraParams(Prefix,StartVal,EndVal,NumPoints,k,Equation,ExtraParams)
 with Form
   if IsNumeric(EndVal) and not IsEmpty(EndVal) and IsNumeric(StartVal) and not IsEmpty(StartVal) then

     ' Avoid errors when calculating log
     If EndVal <= 0 Then
       EndVal = 1
     End If
     If StartVal <= 0 Then
       StartVal = 0.001
     End If

     Log10End = Log10(EndVal)
     Log10Start = Log10(StartVal)

     DeltaLog10 = Log10End - Log10Start 
     IncLog10 = DeltaLog10 / (NumPoints - 1)
     LogVal = Log10Start
      
     ' set defaults for pickup/tap
     pickup = 1
     tap = 1

     'override with ExtraParams
     If Not IsBlankString(ExtraParams) Then
       on error resume next
       CommandList = Replace(ExtraParams, ",", ":")
       call Execute(CommandList)
       on error goto 0 
     End If

     Tdm = k

     Ndx = 0
     for Point = 1 to NumPoints
      
       x = 10 ^ LogVal       
          
       M = x
       I = x
       y = Empty
       on error resume next
       call Execute( Equation )
       on error goto 0 

       'call alog( "point " & Point & "f(" & x &") = " & y )
       if Not IsEmpty(y) then 
         Ndx = Ndx + 1
              
         call SetNdxVar(Prefix+ "__Y",Ndx,y)
         call SetNdxVar(Prefix + "__X",Ndx,x)
              
         '   call alog( Prefix & "__X    XY = " & i & "," & x & "," & y )
       end if 
       LogVal = LogVal + IncLog10 
          
     next
   end if
   call SetVar(Prefix + "__NumCurvePoints",Ndx  )
 end with 
end sub 

'
' ******************************************************************
' Sweep through a user defined equation with a linear x scale 
'
' Kept for backwards compatibility, calls CalcCurveLinearExtraParams()
'
' ******************************************************************
'
sub CalcCurveLinear(Prefix,StartVal,EndVal,NumPoints,k,Equation)
    
    Call CalcCurveLinearExtraParams(Prefix,StartVal,EndVal,NumPoints,k,Equation,"")
end sub

'
' *******************************************************
' Sweep through a user defined equation with a linear x scale 
'
' ExtraParams: Values of extra parameters to be made available to
'              equation. Should be colon (or comma) separated
'
'              eg: "pickup=4.3,xx=12".
'
'              Note: Only pickup and tap are supported for now
' *******************************************************
'
sub CalcCurveLinearExtraParams(Prefix,StartVal,EndVal,NumPoints,k,Equation,ExtraParams)
  with Form
    if IsNumeric(EndVal) and not IsEmpty(EndVal) and IsNumeric(StartVal) and not IsEmpty(StartVal) then
      Delta  = EndVal - StartVal 
      x = StartVal
      Increment = Delta / NumPoints

      ' set defaults for pickup/tap
      pickup = 1
      tap = 1

      'override with ExtraParams
      If Not IsBlankString(ExtraParams) Then
        on error resume next
        CommandList = Replace(ExtraParams, ",", ":")
        call Execute(CommandList)
        on error goto 0 
      End If

      for Point = 1 to NumPoints

        call SetNdxVar(Prefix + "__X",Point,x)
        
        M = x
        I = x
        y = Empty
        on error resume next        
        call Execute( Equation )
        on error goto 0         
        call SetNdxVar(Prefix+ "__Y",Point,y)
        x = x + Increment 
        
      next
    end if

    call SetVar(Prefix + "__NumCurvePoints",NumPoints)
  end with 
end sub 



'
' ************************************************************
' GetInterfaceRev()
' Returns the version of the automation interface
' ************************************************************
function GetInterfaceRev()
    GetInterfaceRev = 0
    on error resume next 
    GetInterfaceRev =  Application.GetInterfaceRev()
    on error goto 0
end function

'
'
' ************************************************************
' CtoF()
' Convert degrees C to degrees F
' ************************************************************
function CtoF( DegC)
    CtoF = Empty
    if not IsEmpty( DegC ) then CtoF =    ToNumber(DegC) * 9.0 / 5.0  + 32.0
end function
'
' ************************************************************
' FtoC()
' Convert degrees F to degrees C
' ************************************************************
function FtoC( DegF)
    FtoC = Empty
    if not IsEmpty( DegF ) then FtoC =  (ToNumber(DegF) - 32) * 5.0 / 9.0  
end function

'
' ************************************************************
' HHMMSStoSecs()
' Convert a HH:MM:SS string to seconds.
' also supports MM:SS and SS
' ************************************************************
'

function HHMMSStoSecs(TimeStr)
    dim parts(3)
    Value = 0
    count = ParseString(TimeStr, parts, ":", 3)
    for i = 1 to count
        Value = Value * 60
        Value = Value +  ToNumber( parts(i) )
    next
    hhMMSStoSecs = Value
end function
    

'
' ************************************************************
' IsChanged()
' Return true if the variable has changed since the last call 
' Use Unique key for multiple is changed for same variable
' ************************************************************
'
function IsChanged(VarName)
      IsChanged = IsChangedEx(VarName,"1")
end function 

function IsChangedEx(VarName,Key)
    IsChangedEx = false
    ChangeVar = "__Changed__" + VarName + Key
    LastValue = GetVar(ChangeVar)
    ThisValue = GetVar(VarName)

    
    
    if IsEmpty(LastValue) then 
        IsChangedEx = true
    else
        if LastValue = "__Empty__" then 
            if not IsEmpty(ThisValue) then IsChangedEx = true
        else 
            if LastValue <> ThisValue then IsChangedEx = true 
        end if 
    end if 
    if IsChangedEx then 
        
        'call alog( "Changed:  " + ChangeVar + "   " + CStr(LastValue) + "    " + CStr(ThisValue) )    
        
        if IsEmpty(ThisValue) then 
            call SetVar(ChangeVar,"__Empty__")
        else 
            call SetVar( ChangeVar,ThisValue)
        end if 
    end if 
end function 



'
'*****************************************************************************
' GetVarTcf( ByVal DegC, ByVal TempVarName, ByVal TableName, ByVal KColumnName )
'
'*****************************************************************************
'	
								  '
FUNCTION GetVarTcf( ByVal DegC, ByVal TempVarName, ByVal TableName, ByVal KColumnName )

' NOTE That this function can not be used on tables with integer temperatures

    SelectStatementHigh = "SELECT TOP 1 " + TempVarName + " as [TempC] , " + KColumnName + " as [KFactor] FROM " + TableName + _
        " WHERE " + TempVarName + "  >= " + Cstr(DegC) + _
        " ORDER BY " + TempVarName 

    SelectStatementLow = "SELECT TOP 1 " + TempVarName + " as [TempC]  , " + KColumnName + " as [KFactor] FROM " + TableName + _
        " WHERE " + TempVarName + "  <= " + Cstr(DegC) + _
        " ORDER BY " + TempVarName + " DESC"


    KFactor = empty


    '
    ' If a valid temperature to lookup
    '
    IF NOT IsEmpty(DegC) AND IsNumeric(DegC) THEN

        '
        ' Get the point above the lookup point
        '
        Form.DeclareVar("db_TempC")
        Form.DeclareVar("db_KFactor")
        call Application.QueryLookupTable(Form.FormId, SelectStatementHigh, 0)
        TempHigh = Form.db_TempC
        KHigh    = Form.db_KFactor

        
        '
        ' Get the factor below the lookup point
        '
        call Application.QueryLookupTable(Form.FormId, SelectStatementLow, 0)
        TempLow = Form.db_TempC
        KLow    = Form.db_KFactor
        
        

        call alog( CStr(TempLow) + "," + CStr(KLow) + "," + CStr(TempHigh) + "," + CStr(KHigh))


        '
        ' If the lookup point was within the table, interpolate the results
        '
        if TempLow <> empty and TempHigh <> empty then
            DeltaT = TempHigh - TempLow
            DeltaK = KHigh - KLow
            if DeltaT > 0 then
                Ratio = (DegC - TempLow) / DeltaT
                KFactor = KLow + ( DeltaK * Ratio )
            else
                KFactor = KLow
            end if

			 Form.DeclareVar("PdbTempCorrFactor")

            '
            ' convert from vt_r4 to string, to vt_r8
            ' to avoid coersion resolution problem.
            '
            StrTcf = CStr(KFactor)
            KFactor = CDbl(StrTcf)
        END IF

    END IF
    GetVarTcf = KFactor
END FUNCTION


'
' *****************************************************
' AddSavedVar()
'
' Wrapper around the application call to ignore for early revs
' *****************************************************
'
sub AddSavedVar(  VarName )
    on error resume next 
    call Application.AddSavedVar(Form.FormId,VarName)
    on error goto 0
end sub 
'
' *****************************************************
' UpdateAsFoundAsLeft(Name)
'
' Sets the Al<Name> variable to the Af<Name> variable if it is 
' empty 
' *****************************************************
'
sub UpdateAsFoundAsLeft(Name)
    AfName = "Af" + Name 
    AlName = "Al" + Name 
    call Application.UpdateAsFoundAsLeft( Form.FormId, AfName,AlName)
end Sub

'
' *****************************************************
' UpdateAsFoundAsLeftNdx(Name,Ndx)
'
' Sets the Al<Name>_<ndx> variable to the Af<Name><ndx> variable if it is 
' empty 
' *****************************************************
'
sub UpdateAsFoundAsLeftNdx(Name, Ndx)
    call UpdateAsFoundAsLeft( Name + "_" + CStr(Ndx) )
end sub 

'
' *****************************************************
' SetNumRows()
'
' Sets the number of rows for the specified template if it has 
' changed 
' *****************************************************
'
sub SetNumRows( TemplateName, NumRows )
        call Application.RunTagCmd ( Form.FormId, TemplateName, "SetNumRows " + CStr(NumRows) )
end sub 

'
' *****************************************************
' SetVisible()
' Shows or hides a template or field 
' *****************************************************
'
sub SetVisible(Name, Visible)
 Cmd = "visible 0"
 FalseStr = CStr(false)
 if not IsEmpty(Visible) then
   VisStr = CStr(Visible)
   if VisStr <> FalseStr and VisStr <> "0" and VisStr <> "-2" then
     Cmd = "visible 1"
   end if
 end if
 call Application.RunTagCmd(Form.FormId, Name, Cmd)
end sub 

'
' *******************************************************
' GetList()
' Reads a list of values from the data tab and returns it
' *******************************************************
'
function GetList(tag)
    GetList = application.ReadDataMultiLineFromTag(Form.FormId, Tag)
end function 

'----------------------------------------------------
' Parses the value returned by application.getVersion
' and constructs the version number.
' eg: If getVersion returns "4.12.TRUNK.2
' the version number is: 4 * 1000 + 12 + 0.2 = 4012.2
'----------------------------------------------------

function getVersion

    on error resume next
    'enable error handling: In case application.getVersion is not defined.
    versionStr = application.getVersion
    'disable error handling
    on error goto 0

    if IsEmpty(versionStr) then
        getVersion = Empty
    else

        ' Check for at most 5 dot (.) separated parts in version string.
        ' NOTE: ParseString() doesn't use the 0'th element of the array.
        '       Therefore, we create an array of 6 elements
        Dim parts(6)
        count = ParseString(versionStr, parts, ".", 5)

        ' Go through the parsed values and get the first 3 numeric values.
        Dim n
        Dim versionNumber
        n = 0

        for i = 1 to count
            str = parts(i)

            if IsNumeric(str) then
                n = n + 1

                ' If the first 3 numbers are 4,12,5, The version will be 4012.5
                select case n
                    case 1
                        versionNumber = 1000 * CInt(str)
                    case 2
                        versionNumber = versionNumber + CInt(str)
                    case 3
                        
                        'The third number is the decimal part
                        thirdpart = CInt(str)
                        thirdpartdivisor = 10
                        if thirdpart >= 10 then
                            thirdpartdivisor = 100
                        end if
                        versionNumber = CDbl(versionNumber) + (CDbl(thirdpart) / CDbl(thirdpartdivisor))
                        ' the line below does not work on systems where comma is the decimal point (replaced with line above)
                        ' versionNumber = Cdbl(versionNumber & "." & str)
                    case else
                        exit for
                end select
            end if
        next
        
        if InStr( versionStr,"trunk") > 0  then versionNumber = versionNumber + 500
            

        getVersion = versionNumber

    end if
end function


function Translate(Str)
    ' handle error if TranslateStr is not defined
    on error resume next
    Translate = Application.TranslateStr(Str)
    on error goto 0

    if IsEmpty(Translate) then
        Translate = Str
    end if
end function

'
'Returns input number rounded to specified number of significant figures.
'
Function RoundSF(dblInput, intSF)

    if IsEmpty(dblInput) OR dblInput = 0 then
        RoundSF = dblInput
    else

        '-- Store sign of dblInput --
        intSign = Sgn(dblInput)

        '-- Calculate exponent of dblInput --
        intCorrPower = Int(Log10(Abs(dblInput)))

        RoundSF = Round(dblInput * 10 ^ ((intSF - 1) - intCorrPower))   'integer value with no sig fig
        RoundSF = RoundSF * 10 ^ (intCorrPower - (intSF - 1))         'raise to original power

        '-- Reconsitute final answer --
        RoundSF = RoundSF * intSign
    end if
End Function

Function Log10(Value)
    Log10 = Log(Value) / Log(10)
End Function

' Returns min of 3 numbers, ignoring empty values
' If all 3 numbers are Empty, Empty is returned
Function Min3(A, B, C)

    Dim Values(3)
    Values(0)=A
    Values(1)=B
    Values(2)=C

    Smallest = Empty

    For Each Val In Values
        If Not IsEmpty(Val) then        
            If IsEmpty(Smallest) OR Smallest > Val then
                Smallest = Val
            End If
        End If
    Next
    Min3=Smallest

End Function

'
' Function GetChartRGBColor()
' Return an RGB color value definition based on the index. The index is used to return unique color
' values on different calls to GetChartRGBColor.
' Index should be greater than 0
'
Function GetChartRGBColor(Index)
    ' Color Definitions
    ' BLUE,    RED,      GREEN,    PURPLE, 
    ' CYAN,    YELLOW,   MAGENTA,  VIOLET
    ' ORANGE,  BROWN,    CORAL,    MAROON, 
    ' OLIVE,   TEAL,     SEAGREEN, LIME,
    ' LTGREEN, MIDNIGHT, SKY,      BEIGE,
    ' GRAY10,  GRAY20,   GRAY30,   GRAY40,
    ' GRAY50,  GRAY60,   GRAY70,   GRAY80,	     
    ' GRAY90,  LTRED,    BLACK
    
    
    

    ChartColors = Array( "R0,G0,B255",     "R255,G0,B0",     "R0,G255,B0",     "R128,G0,B128", _
                    "R0,G255,B255",   "R255,G255,B0",   "R255,G0,B255",   "R128,G0,B255", _
                    "R255,G102,B0",   "R153,G51,B0",    "R255,G0,B128",   "R128,G0,B0", _
                    "R128,G128,B0",   "R0,G128,B128",   "R0,G255,B128",   "R0,G255,B0", _
                    "R128,G255,B0",   "R0,G0,B128",     "R0,G128,B255",   "R255,G128,B0", _
                    "R230,G230,B230", "R205,G205,B205", "R179,G179,B179", "R152,G152,B152", _
                    "R128,G128,B128", "R102,G102,B102", "R76,G76,B76",    "R51,G51,B51", _
                    "R25,G25,B25",    "R255,G92,B92",   "R0,G0,B0" )


    if GetVar("NoYellowTraces") = 1 then 
        ChartColors(5) = "R0,G0,B0" 
    end if
    
    if GetVar("NoGreenTraces") = 1 then 
        ChartColors(2) = "R128,G128,B0" 
    end if 
 
    GetChartRGBColor = ChartColors( ( index - 1 ) Mod 31) 
End Function

'
' Function GetChartLineType()
' Return an Chart line type based on the index. The index is used to return different line type
' values on different calls to GetChartRGBColor.
' Default is a Line type string that will be returned once the line types run out.
' If Default is Empty, the line types will cycle through the available list.
' Index should be greater than 0
'
Function GetChartLineType(Index, Default)
    LineTypes = Array("DASH", "DOT", "DASHDOT", "DASHDOTDOT", "SOLID" )
    Dim SelectedLine

    If ( index > 5 ) Then
	If IsEmpty ( Default )  Then
	    SelectedLine = LineTypes ( ( index - 1 ) Mod 5 )
	Else
	    SelectedLine = Default
	End If
    Else
	SelectedLine = LineTypes ( index - 1 )
    End If

    GetChartLineType = SelectedLine
End Function

'
' Function GetChartSymbol()
' Return an Chart symbol based on the index. The index is used to return different symbol 
' values on different calls to GetChartRGBColor.
' Default is a symbol string that will be returned once the symbols run out.
' If Default is Empty, the symbols will cycle through the available list.
' Index should be greater than 0
'
Function GetChartSymbol(Index, Default)
    Dim SelectedSymbol
    Symbols = Array("SYM_CIRCLE","SYM_UPTRIANGLE","SYM_DOWNTRIANGLE","SYM_SQUARE","SYM_DIAMOND")

    If ( index > 5 ) Then
	If IsEmpty ( Default )  Then
	    SelectedSymbol = Symbols( ( index - 1 ) Mod 5)
	Else
	    SelectedSymbol = Default
	End If
    Else
	SelectedSymbol = Symbols( index - 1 )
    End If

    GetChartSymbol = SelectedSymbol
End Function

' 
' Function GetChartCommands()
'
' Returns a chart command string that can be passed to a chart via _cmd to dynamically draw traces.
' The traces will have different colors and line types.
'
' Parameters:
'
' TraceName:
'           Used to set the Trace Name. If more than one trace is generated, The Trace Name will
'           have the trace number appended to it. Eg: TraceName = "Impedence", the trace names 
'           will be "Impedence 1", "Impedence 2" etc.
'
' StartTraceNum:
'	    The trace number to start at. Eg: If the chart already has one trace on it, trace
'           number can be set to 2 to avoid overwriting that trace. 
'
' XRange: 
'           The x range definition. Eg: Cell_#!1-NumCells 
'
' YRange: 
'           The y range definition. Eg: Impedence#!1-NumCells
'
' NumTraces:
'           The number of traces to generate.
'
' ScaleGroup:
'           The scale group for the trace. (should be 1 or 2)
'
' IncludeSymbols:
'           Whether symbols should be included in the trace
'
'Notes on XRange and YRange: 
'   1) Don't use the : character.
'   2) Any %TRACE_NUM% in the Range definition strings will be replaced by the trace number.
'      eg: If YRange= "db_Impedence_#_%TRACE_NUM%!1-NumCells",  the first trace will have a
'      YRange of "db_Impedence_#_1!1-NumCells" and the second YRange will have 
'      "db_Impedence_#_2!1-NumCells" and so on.
'
'''''''''''''''''''''''
' Trace Command Format:
' 
' The trace command format accepted by chart_cmd is:
'
' TRC=Trace Number,Trace Name,X Range,Y Range, R255, G255, B122, Point Type, Line Type, Scale Group 
'
' X Range format: $VarName_#!Start-End.  Start and End can be constants or variables.
'                 The dollar sign prefix means the tag values will be strings.
'
' Y Range format: Similar to the X Range format, but $ is not supported.
'
' Point Type: one of SYM_CIRCLE,SYM_NONE,SYM_CIRCLE,SYM_UPTRIANGLE,SYM_DOWNTRIANGLE,SYM_SQUARE,SYM_DIAMOND
' Line Type: one of SOLID, DASHDOTDOT, DASHDOT, DOT, DASH
' Scale Group: 1 or 2
'
Function GetChartTraceCommands(TraceName, StartTraceNum, XRange, YRange, NumTraces, ScaleGroup, IncludeSymbols)
    ChartCmd = ""

    TraceNum = StartTraceNum
    
    For i = 1 To NumTraces

	ColorStr = GetChartRGBColor(i)

        If IncludeSymbols Then
            Symbol = GetChartSymbol(i, Empty)
        Else
      	    Symbol = "SYM_NONE"
        End If
	
	' Go through line types and default to "SOLID" once the types run out.
	LineType = GetChartLineType(i, "SOLID")

    TraceOffset = GetVar("TraceOffset")
	XSeries = Replace(XRange, "%TRACE_NUM%", CStr(i) + TraceOffset)
	YSeries = Replace(YRange, "%TRACE_NUM%", CStr(i) + TraceOffset)

	ChartCmd = ChartCmd & "TRC=" & TraceNum & "," & TraceName & i & " ," & XSeries & "," & YSeries & _
	                    "," & ColorStr & "," & Symbol & "," & LineType & "," & ScaleGroup & ":"

	TraceNum = TraceNum + 1
    Next

    GetChartTraceCommands=ChartCmd	  
End Function

sub PostOnUpdate()
    DefList = GetVar("AutoDeficiencyList")
    if DefList <> Empty then 
        DefList = "Test(s) " + DefList + " Failed"
    else 
        DefList = " "
    end if 

  '  call SetDeficienciesInternal( "Comments1" , DefList)
end sub 
'
' Calculate the error for the field 'ErrTag' 
' if the error is larger than Tolerance, color it red and add it to the deficiancy list
'
sub CalcErrorTag( ErrTag, Actual,Desired,Tolerance,DefNote)
    FgColor = CLR_BLACK
    ErrorVal = Empty
    if IsNumeric(Actual) and IsNumeric(Desired) then
        if not IsEmpty(Actual) and not IsEmpty(Desired) and Desired<> 0 then
            ErrorVal  = abs( Actual - Desired ) / Desired * 100 
            if ErrorVal > Tolerance then 
                Call AddFailure(DefNote)
                FgColor = CLR_RED
            end if 
        end if 
    end if 
    call SetVar(ErrTag,ErrorVal)
    call Application.RunTagCmd(Form.FormId, ErrTag, "FgColor   " + CStr(FgColor) )
end sub 
'
' ***********************************************************************************
' Calculate the error for the field 'ErrTag' based on the 'ActualTag' and 'DesiredTag'
'
' if the error is larger than Tolerance, color it red and add it to the deficiency list and use the 
' 'DefNote' to anotate the deficiency
' 
'
' Do this for tags with a sufix of the range 'StartNdx' to 'EndNdx'
'
' **********************************************************************************
'
sub CalcErrorRange( ErrTag, ActualTag,DesiredTag,Tolerance,DefNote,StartNdx, EndNdx)
    
    Msg = ""
    bFailed = false 
    for Ndx = StartNdx to EndNdx
        FgColor = CLR_BLACK
        ErrorVal = Empty
        Actual = GetNdxVar(ActualTag,Ndx)
        Desired = GetNdxVar(DesiredTag,Ndx)
        
        Msg = Msg + CStr(Actual) + "  xxx " + CStr(Desired)
        
        if IsNumeric(Actual) and IsNumeric(Desired) then
            if not IsEmpty(Actual) and not IsEmpty(Desired) and Desired<> 0 then
                ErrorVal  = abs( Actual - Desired ) / Desired * 100 
                if ErrorVal > Tolerance then 
                    bFailed = true 
                    FgColor = CLR_RED
                end if 
            end if 
    end if 
       NdxErrTag = ErrTag + "_" + CStr(Ndx)
        call SetVar(NdxErrTag ,ErrorVal)
        call Application.RunTagCmd(Form.FormId, NdxErrTag , "FgColor   " + CStr(FgColor) )
    next 
    '
    ' If any one of the test fail, flag the overall test as a failure 
    '
    if bFailed then 
        Call AddFailure(DefNote)
    end if 

end sub 

            
            '
' ***********************************************************************************
' Calculate the error for the field 'Value' based on the min and max values 
'
' if the error is larger than Tolerance, color it red and add it to the deficiency list and use the 
' 'DefNote' to anotate the deficiency
' 
'
'
' **********************************************************************************
'
function  CheckError( ActualTag, MinVal,MaxVal,DefNote)
    
    
    
    CheckError = true
    
    MaxDepth = GetVar("FormMaxCalcDepth")
    if IsEmpty(MaxDepth) then MaxDepth = 1
    '
    ' Only do for the last evaluation depth
    '
    if Form.FormCurCalcDepth = MaxDepth  then
        
        Msg = ""
        FgColor = CLR_BLACK

        Actual = GetVar(ActualTag)
                
        if IsNumeric(MinVal) and IsNumeric(MaxVal) and IsNumeric(Actual) then
            if not IsEmpty(MinVal) and not IsEmpty(MaxVal) and not IsEmpty( Actual )  then
                if Actual < MinVal or Actual > MaxVal then 
                    CheckError = false 
                    FgColor = CLR_RED
                    
                    if not IsBlankString( DefNote ) then 
                        Call AddFailure(DefNote) 
                    end if 
                end if 
            end if 
        end if 
        call Application.RunTagCmd(Form.FormId, ActualTag , "FgColor   " + CStr(FgColor) )
   end if 
end function 

'
' ***********************************************************************************
' Calculate the minimum value using both a percentage and an absolute offset 
'
' **********************************************************************************
'

sub CalcMinMax(MinTag, MaxTag, Theo, PercentVal, AbsVal )
    CalcMin = empty
    CalcMax = empty 
    if IsNumeric( Theo) and IsNumeric(PercentVal) and IsNumeric(AbsVal) then
        if not IsEmpty(Theo) and not IsEmpty( PercentVal ) and Not isEmpty( AbsVal) then
            CalcMin = Theo * ( 1 - (PercentVal / 100) ) - AbsVal 
            CalcMax = Theo * ( 1 +(PercentVal / 100) ) + AbsVal 
        end if 
    end if 
    call SetVar( MinTag, CalcMin )
    call SetVar(MaxTag,CalcMax)
end sub 

'
' ***********************************************************************************
' Calculate the minimum value using both a percentage and an absolute offset with plus and minus
' Set the def note and change the color to red if out of tolerance
'
' Returns true if it passed
'
' **********************************************************************************
'
function CalcMinMaxAndErrorPm(ResultTag, MinTag, MaxTag, Theo , PercentPlus,PercentMinus, AbsPlus,AbsMinus, DefNote )

    CalcMin = Empty
    CalcMax = Empty
    
    if IsEmpty( AbsPlus ) then AbsPlus = 0
    if IsEmpty( PercentMinus ) then PercentMinus = PercentPlus
    if IsEmpty( AbsMinus ) then AbsMinus = AbsPlus

    if IsNumeric( Theo) and IsNumeric(PercentMinus) and IsNumeric(AbsPlus) then
        if not IsEmpty(Theo) and not IsEmpty( PercentPlus )  then
            CalcMin = Theo * ( 1 - (PercentMinus / 100) ) - AbsMinus 
            CalcMax = Theo * ( 1 +(PercentPlus / 100) ) + AbsPlus
        end if 
    end if 
    call SetVar( MinTag, CalcMin )
    call SetVar(MaxTag,CalcMax)
    
    CalcMinMaxAndErrorPm = CheckError(ResultTag, CalcMin, CalcMax, DefNote)
    
end function



sub CalcMinMaxAndError(MinTag, MaxTag, Theo, PercentVal, AbsVal, AfTag, AlTag, DefNote )
    
    call CalcMinMax(MinTag, MaxTag, Theo, PercentVal, AbsVal )
    call CheckError(AfTag, GetVar(MinTag), GetVar(MaxTag), "")
    call CheckError(AlTag, GetVar(MinTag), GetVar(MaxTag), DefNote)    
    
end sub



sub CalcErrorAbsRange( ErrTag, ActualTag,DesiredTag,Tolerance, AbsTolerance, DefNote,StartNdx, EndNdx)
    
    Msg = ""
    bFailed = false 
    for Ndx = StartNdx to EndNdx
        FgColor = CLR_BLACK
        ErrorVal = Empty
        Actual = GetNdxVar(ActualTag,Ndx)
        Desired = GetNdxVar(DesiredTag,Ndx)
        
         call CalcMinMax("TmpMin", "TmpMax", Desired, Tolerance, AbsTolerance )
         
         TmpMin = GetVar("TmpMin")
         TmpMax = GetVar("TmpMax")
        
        Msg = Msg + CStr(Actual) + "  xxx " + CStr(Desired)
        
        if IsNumeric(Actual) and IsNumeric(Desired) then
            if not IsEmpty(Actual) and not IsEmpty(Desired) and Desired<> 0 then
                ErrorVal  = abs( Actual - Desired ) / Desired * 100 
                if Actual > TmpMax or Actual < TmpMin then 
                    bFailed = true 
                    FgColor = CLR_RED
                end if 
            end if 
    end if 
       NdxErrTag = ErrTag + "_" + CStr(Ndx)
        call SetVar(NdxErrTag ,ErrorVal)
        call Application.RunTagCmd(Form.FormId, NdxErrTag , "FgColor   " + CStr(FgColor) )
    next 
    '
    ' If any one of the test fail, flag the overall test as a failure 
    '
    if bFailed then 
        Call AddFailure(DefNote)
    end if 

end sub 


function MakeStrForRunTagCmd(TagName)

    RetVal = GetVar(TagName)
    RetVal = Replace(RetVal, "'", "''")
    
    MakeStrForRunTagCmd = "'" + RetVal + "'"
    
end function
'*****************************************************************************
'
' FUNCTION GetNdxVarMidEx(StrA,Ndx,StrB)
'
'  This function returns the value of a variable with and index in the middle
'
'  If the _1_ does not exist, check _ (for single record doble values) 
'
'*****************************************************************************
function GetNdxVarMidEx(StrAIn,Ndx,StrB)
    Stra = StrAIn
    GetNdxVarMidEx = ""
    Name = StrA + CStr(Ndx) + StrB
    GetNdxVarMidEx  = GetVar(Name)
    if GetNdxVarMidEx = Empty and Ndx = 1 then 
        LenA = Len(StrA)
        if LenA > 2 then 
            Stra = Left(Stra,LenA-1)
            Name = StrA + StrB
call alog("Double check " + Name)            
            GetNdxVarMidEx  = GetVar(Name)
        end if 
    end if 
end function

'
' *****************************************
' AddFailure()
' Add a failure note to the auto deficiencies section
' *****************************************
'
SUB AddFailure(DefNote)     
    DefList = GetVar("AutoDeficiencyList")               
    MaxDepth = GetVar("FormMaxCalcDepth")
    if IsEmpty(MaxDepth) then MaxDepth = 1
    if DefList <> Empty then DefList = DefList + ","     
    DefList = DefList + DefNote                           
    if Form.FormCurCalcDepth = MaxDepth-1  then        
        Call SetVar("AutoDeficiencyList",DefList)           
    end if 
END SUB              

'
' *****************************************
' GetHistoryData()
' Get the historical data for the tags into the variables
' db_<variableName>_#.  It will also update the 
' db_<variableName>_0 record with the tag value 
' to plot the present results with the historical values
'
' it also updates.. 
'    db_<VariableName>_Avg
'    db_<VariableName>_Min
'    db_<VariableName>_Max
'    db_<VariableName>_Std
'
' *****************************************
'
sub GetHistoryData( Tags )
    
    if Form.FormLoadedValues and GetVar("__HistoryLoaded") <> 1 then
        call setVar("__HistoryLoaded",1)
        ' 
        ' Null any old records
        ' 
        Dim parts(100)
        count = ParseString(Tags, parts, ",", 100)
        
        for i = 1 to Count
            for History = 1 to 10 
                call SetNdxVar("db_" + parts(i),History,Empty)
            next 
        next
        
        call application.GetHistDataForTags(form.formid,Tags)
        
    end if 

    '
    ' Update any newly entered tags to the db_<TagName>_0 
    ' variable 
    call UpdateDbZero(Tags)



end sub 
                                    
'
' *****************************************
' UpdateDbZero()
' Updates the db_<variableName>_0 records with 
' the tag values to plot the present results with the 
' historical values
' *****************************************
'
sub UpdateDbZero( Tags )
        Dim parts(100)
        count = ParseString(Tags, parts, ",", 100)
        
        call alog("Count = " + CStr(Count) )
        for i = 1 to Count
            Tag = Parts(i)
            Value = GetVar(Tag)
            call SetNdxVar("db_" + Tag,0,Value)
        next
end sub 

'
' ************************************************************
' HHMMtoSecs()
' Convert a HH:MM string to seconds.
' ************************************************************
'

function HHMMtoSecs(TimeStr)
    dim parts(2)
    Value = 0
    count = ParseString(TimeStr, parts, ":", 2)    
    for i = 1 to count
        Value = Value +  ToNumber( parts(i) )
        Value = Value * 60
    next
    hhMMtoSecs = Value
end function

   
'**********************************************************************
FUNCTION SecsToHHMMSS( TimeInSeconds )
'**********************************************************************
    
    TimeInSeconds =  CLng( ToNumber( TimeInSeconds ) )
    HH = Int( Divide( TimeInSeconds, 3600 ) )
    MM = TimeInSeconds Mod 3600
    SS = MM Mod 60
    MM = Int( Divide( MM, 60 ) )
    
    SecsToHHMMSS = right("0"& HH,2) & ":" & right("0"& MM,2) & ":" & right("0"& SS,2)
    
END FUNCTION

'**********************************************************************
FUNCTION FormatTimeHHMMSS( TimeStr )
'   Converts a time string into HH:MM:SS format
'   If the input string contains colon(s), it is treated as HH:MM(:SS) format
'   Otherwise, the input is assumed to be a value in seconds
'**********************************************************************
    if ( InStr( TimeStr, ":" ) ) then
        
        HHStr = Left( TimeStr, InStr( TimeStr, ":" ) - 1 )
        TimeStr = Mid( TimeStr, InStr( TimeStr, ":" ) + 1 )
        
        if( InStr( TimeStr, ":" ) ) then
            MMStr = Left( TimeStr, InStr( TimeStr, ":") - 1 )
            SSStr = Mid( TimeStr, InStr( TimeStr, ":" ) + 1 )
        else
            MMStr = TimeStr
            SSStr = "00"
        end if

        HH = Int( ToNumber( HHStr ) )
        MM = Int( ToNumber( MMStr ) )
        SS = Int( ToNumber( SSStr ) )
                
        if( SS > 59 ) then
            MM = MM + CInt( Divide( SS, 60 ) )
            SS = SS Mod 60
        end if
        if( SS < 0 ) then
            SS = 0
        end if

        if( MM > 59 ) then
            HH = HH + CInt( Divide( MM, 60 ) )
            MM = MM Mod 60
        end if
        if( MM < 0 ) then
            MM = 0
        end if
    
        if( HH > 99 ) then
            HH = 99
            MM = 59
            SS = 59
        end if
        if( HH < 0 ) then
            HH = 0
        end if  
      
        HHStr = right("0"& HH,2)
        MMStr = right("0"& MM,2)
        SSStr = right("0"& SS,2)
            
        FormatTimeHHMMSS = HHStr & ":" & MMStr & ":" & SSStr

    else
        
        FormatTimeHHMMSS = SecsToHHMMSS( TimeStr )
        
    end if

END FUNCTION

'*****************************************************************************
'
' SUB AddDeficiency()
'
' This method adds a deficiency note to the auto deficiency line.
' The note is added unchanged, and no "Failed" text is appended as is done
' by AddFailure().
'
' Temporarily added to this template, to support older versions that do-not have this
' change in StandardScripts.h
'
'*****************************************************************************
SUB AddDeficiency(DefNote)
    DefList = GetVar("DirectDeficiencyList")
    if DefList <> Empty then DefList = DefList & ",\n"
        
    DefList = DefList & DefNote
 
    MaxDepth = GetVar("FormMaxCalcDepth")
    if IsEmpty(MaxDepth) then MaxDepth = 1   
    if Form.FormCurCalcDepth = MaxDepth-1 then
         Call SetVar( "DirectDeficiencyList",DefList)
    End if
END SUB

FUNCTION HaveAutoDefs()
    Have = false    

        PdbAutoDefs = GetVar("PdbAutoDefs")
        AutoDeficiencyList = GetVar("AutoDeficiencyList")
        DirectDeficiencyList = GetVar("DirectDeficiencyList")
       ' Call alog("In HaveAuto 1: " & AutoDeficiencyList)
       ' Call alog("In HaveAuto 2: " & PdbAutoDefs)
       ' Call alog("In HaveAuto 3: " & DirectDeficiencyList)
        If Not IsBlankString(PdbAutoDefs) OR Not IsBlankString(AutoDeficiencyList) OR Not IsBlankString(DirectDeficiencyList) Then
            Have = true
        End If
        
    HaveAutoDefs = Have
     
END FUNCTION

'*****************************************************************************
'
' SUB PostOnUpdate2()
'
' Overrides the default PostOnUpdate2() by adding support for "DirectDeficiencyList"
'
' Temporarily added to this template, to support older versions that do-not have this
' change in StandardScripts.h
'
'*****************************************************************************
SUB PostOnUpdate2NotAnymore()
   DispId = Form.GetDispId("AutoDeficiency_1")
   if DispId > 0 then
     DefList = GetVar("AutoDeficiencyList")
     DefList2 = GetVar("DefList")
     
     if DefList <> Empty and DefList2 <> Empty then DefList = DefList & ",\n"
     DefList =  DefList & DefList2

     If DefList <> Empty  Then
         DefList = DefList + TLang(" Failed")
     End If
     
     DefList3 = GetVar("DirectDeficiencyList")     
     if DefList <> Empty  and DefList3 <> Empty then DefList = DefList & ",\n"
     DefList =  DefList & DefList3
     
     DefList4 = GetVar("PdbAutoDefs")     
     if DefList <> Empty  and DefList4 <> Empty then DefList = DefList & ",\n"
     DefList =  DefList & DefList4
     
     PdbVersion = GetVersion() 
     
     If DefList <> Empty Then
         If PdbVersion >= 8000.27 then             
             Call Application.RunTagCmd(Form.FormId, "AutoDeficiency_1", "WrapTextToTable '" & DefList & "'" )
         Else
             Call Application.RunTagCmd(Form.FormId, "AutoDeficiency_1", "SetWrappedText '" & DefList & "'" )         
         End If       
          
    End If      
 
      
    if DefList =  Empty then call SetVar( "AutoDeficiency_1", Empty )
      
else
       Call PostOnUpdate()
   end if
   'call alog( "Testing DispId = " & DispId &  "  deflist = " & DefList )
END SUB

'*****************************************************************************
'
' SUB PreOnUpdate()
'
' Overrides the default PreOnUpdate() by adding support for "DirectDeficiencyList"
'
' Temporarily added to this template, to support older versions that do-not have this
' change in StandardScripts.h
'
'*****************************************************************************
SUB PreOnUpdate()
    call SetVar("AutoDeficiencyList",Empty)
    call SetVar("DirectDeficiencyList",Empty)
END SUB

'*****************************************************************************
'   FUNCTION GetMin()
'     
'   Returns the lesser of two numbers
'
'*****************************************************************************
FUNCTION GetMin( Num1, Num2 )
    
    if( Num1 <= Num2 ) then
        GetMin = Num1
    else
        GetMin = Num2
    end if
    
END FUNCTION

'*****************************************************************************
'   FUNCTION GetMax()
'     
'   Returns the greater of two numbers
'
'*****************************************************************************
FUNCTION GetMax( Num1, Num2 )
    if( Num1 >= Num2 ) then
        GetMax = Num1
    else
        GetMax = Num2
    end if
         
END FUNCTION

FUNCTION GetHdrTempUnit()

  bFahrenheit = true

  '
  ' Check the old string unit if it defined, otherwise use the new integer unit.
  ' 
  htuStr = GetVar("HdrTempUnit")
  if IsEmpty(htuStr) then
    if (GetVar("HdrTempUnitInt") = GetVar("HdrTempUnitIntC")) then
      bFahrenheit = false
    end if
  else
    if InStr(htuStr, "C") then 
      bFahrenheit = false
    end if
  end if

  if bFahrenheit then
    GetHdrTempUnit = "F"
  else
    GetHdrTempUnit = "C"
  end if

END FUNCTION

'************************************
' SetTemperatureC
'
' Changes temperature to the given value (in c)
'
' Handles conversion to F if needed
'
'************************************
sub SetTemperatureC(Temp)

    Dim TempF, TempC, TempUnit
       
    TempF = FormatNumber( CtoF( Temp ),1)
    TempC = Temp        
        
    call SetVar("Temperature", TempF)
    call SetVar("TemperatureC", TempC)

    TempUnit = GetHdrTempUnit()
        
    if InStr(TempUnit, "C") then 
      call SetVar("EnterTemp", TempC)
    else
      call SetVar("EnterTemp", TempF)
    end if
    
end sub

'************************************
' SetTemperatureF
'
' Changes temperature to the given value (in F)
'
' Handles conversion to C if needed
'
'************************************
sub SetTemperatureF(Temp)
    
    Dim TempF, TempC, TempUnit

    TempF = Temp
    TempC = FormatNumber( FtoC( Temp ),1)
        
    call SetVar("Temperature", TempF)
    call SetVar("TemperatureC", TempC)

    TempUnit = GetHdrTempUnit()
    
    if InStr( TempUnit , "C" ) then 
        call SetVar("EnterTemp", TempC)
    else
        call SetVar("EnterTemp", TempF)
    end if
    
end sub

'************************************
' GetTemperatureC
'
' Get the temperature in C
'
' Handles conversion from F if needed
'
'************************************
function GetTemperatureC()

    Dim TempC, TempUnit
          
    TempUnit = GetHdrTempUnit()
    EnterTemp = GetVar("EnterTemp")
    
    if IsBlankString(TempUnit) And IsBlankString(EnterTemp) Then
        
        TempC = FtoC( GetVar("Temperature") )            
        
    elseif InStr( TempUnit , "C" ) then
        TempC = EnterTemp
    else
        TempC = FtoC( EnterTemp )
    end if
    
    GetTemperatureC = TempC
    
end function

'************************************
' GetTemperatureF
'
' Get temperature value in F
'
' Handles conversion from C if needed
'
'************************************
function GetTemperatureF()

    Dim TempF, TempUnit
          
    TempUnit = GetHdrTempUnit()
    EnterTemp = GetVar("EnterTemp")
    
    if IsBlankString(TempUnit) And IsBlankString(EnterTemp) Then
        
        TempF = GetVar("Temperature")
        
    elseif InStr( TempUnit , "C" ) then
        TempF = CtoF( EnterTemp )
    else
        TempF = EnterTemp
    end if
    
end function

'*****************************************************************************
'
' DoModalFormWithReturn
'
'  Loads a form in a modal window, then pauses script until the child form closes
'
'
'*****************************************************************************

SUB DoModalFormWithReturn( FormName )

    ParentFormId = GetVar( "FormId" )

    call application.doModalForm( FormName )

    do
       sleep(1)
    Loop while (GetVar("FormId") = ParentFormId)

    do
       sleep(1)      
    Loop while (GetVar("FormId") <> ParentFormId)

END SUB 

'*******************************************************************
' Sub MapTags(TagMap)
'
' Maps existing tags to new tag names based on passed tag map.
'
' Parameters:
'      TagMap: Name of tag map available in data tag. Eg: "TeggTagMap:"
'              Needs to include the colon (:).
'
'*******************************************************************
Sub MapTags(TagMap)

    Dim Lines, StartNdx, EndNdx, SrcTag, DestTag, Parts, SrcVal
    

    TegTagMap = Application.ReadDataMultiLineFromTag(Form.FormId, TagMap)
    Lines = Split(TegTagMap, vbLf)

    StartNdx = LBound(Lines)
    EndNdx = UBound(Lines)

    For Ndx = StartNdx To EndNdx

        Parts = Split(Lines(Ndx), ",")
        If UBound(Parts) >= 1 Then
            SrcTag = Trim(Parts(0))
            DestTag = Trim(Parts(1))
            
            If Not IsBlankString(SrcTag) and Not IsBlankString(DestTag) Then
                SrcVal = GetVar(SrcTag)
                'Call aLog("SrcVal = " & SrcVal & ", DestTag = " & DestTag & ", SrcTag = " & SrcTag)
                Call SetVar(DestTag, SrcVal)
                
            End If            
            
        End If
    Next

End Sub

'*******************************************************************
' Sub Toggle()
'   Swap Tagname value between 1 and 0
'*******************************************************************
sub Toggle(Tag)
    val = GetVar(Tag)
    if val then 
        val = 0
    else
        val = 1
    end if 
    call SetVar(Tag,val)
end sub 

'*******************************************************************
' Sub Sink()
'   Display a button as sunken
'*******************************************************************
sub Sink( Tags , State )
    
    if State then 
        Cmd = "Sink 2 , " & CLR_YELLOW
    else
        Cmd = "Sink 1, " & CLR_GRAY20
    end if 
    call RunTagCmd(Tags,Cmd)
end sub 

'*******************************************************************
' function GetRowFromTag()
'*******************************************************************
function GetRowFromTag(TagName)
  GetRowFromTag = 0
  Length = Len(TagName)
  if Length > 0 then
    for ndx = 1 to Length
        if Mid(TagName,Ndx,1) = "_" then
            UnderScoreNdx = Ndx
        end if
    next
  end if
  
  if UnderScoreNdx > 0 and UnderScoreNdx < Length then
	RestOfStr = Mid( TagName,UnderScoreNdx+1)
    RowNdx  = ToNumber( RestOfStr )
    if RowNdx < 1 then RowNdx = 1
    GetRowFromTag = RowNdx
  end if
end function

'*******************************************************************
' function acos(a)
'*******************************************************************

function Acos(a)
   if Abs(a)=1 then 
       Acos = (1-a)*PI/2
   else 
       alog("a = " & a )
       Acos = Atn(-a/sqr( 1-a*a ) )+2*Atn(1)
    end if
end function

'*******************************************************************
' function PolarDegreesToCartesian
'   Converts polar coords to cartesian
'   Returns true if conversion is successful
'*******************************************************************

function PolarDegreesToCartesian( dMag, dDeg, byref dX, byref dY )
    
    PolarDegreesToCartesian = false
    
    if NOT (IsEmpty(dMag) OR IsEmpty(dDeg)) then
        dX = dMag * cosd(dDeg)
        dY = dMag * sind(dDeg)
        
        PolarDegreesToCartesian = true
    end if
    
end function

'*******************************************************************
' function CartesianToPolarDegrees
'   Converts polar coords to cartesian
'   Returns true if conversion is successful
'*******************************************************************

function CartesianToPolarDegrees( dX, dY, byref dMag, byref dDeg )
    
    CartesianToPolarDegrees = false
    
    if NOT (IsEmpty(dX) OR IsEmpty(dY)) then
        
        dMag = sqr( (dX * dX) + (dY * dY) )
        dRad = 0.0

        if ( dMag <> 0.0 ) then
            dRad = Acos( dX / dMag )
            
            if ( dY < 0.0 ) then
                dRad = 2.0 * GetPI() - dRad
            end if
        end if
        
        dDeg = (dRad * 180.0) / GetPI()
                
        CartesianToPolarDegrees = true
    end if
    
end function


'*******************************************************************
' function ConvertCartesianToPolarDegrees
'   Converts to/from polar coords to cartesian
'   ToPolar - indicated if we are going to/from polar
'   Returns true if conversion is successful
'*******************************************************************

function ConvertCartesianToPolarDegrees( MagOrX, AngOrY, byref MagOrXOut, byref AngOrYOut, ToPolar )
    
    if ( ToPolar ) then
        ConvertCartesianToPolarDegrees = CartesianToPolarDegrees( MagOrX, AngOrY, MagOrXOut, AngOrYOut )
    else
        ConvertCartesianToPolarDegrees = PolarDegreesToCartesian( MagOrX, AngOrY, MagOrXOut, AngOrYOut )
    end if    
    
end function


'*****************************************************************
' Adds auto deficiency message for given tag. 
'
' First checks if corresponding automation call is available, and 
' if not uses older AddDeficiency function
'
'*****************************************************************
Sub AddAutoDeficiency(Tag, Message)
    
    ' Using error handling instead of using a version check
    ' for speed, since GetVersion() results in an automation call 
    '
    On Error Resume Next
    Call Application.AddAutoDeficiency(Form.FormId, Tag, Message)
    If Err.Number > 0 Then
        Call AddDeficiency(Message)
        Err.Clear
    End If
    
    On Error Goto 0
End Sub

'*****************************************************************
' Clears auto deficiency message for given tag. 
'
' Only run if corresponding automation call is available
'
'*****************************************************************
Sub ClearAutoDeficiency(Tag)
    On Error Resume Next
    Call Application.ClearAutoDeficiency(Form.FormId, Tag)
    On Error Goto 0    
End Sub

'

' **************************************************

' InitEnumeration()

' Creates variables for a sequential enumeration 

' EnumList = "RM_,a,b,c,d"

' creates the following

' RM_A = 0,  RM_B = 1, RM_C = 2 , RM_D = 3

' **************************************************

'

sub InitEnumeration( EnumList )

    dim List(100)

    Size = ParseString( EnumList,List,",",100)

    Prefix = List(1)

    for i = 2 to Size

        call SetVar(Prefix & List(i) , i - 2 )

'        call alog("InitEnum " & Prefix & List(i) )

    next

end sub


' *******************************************************
' GetDegreeSym()
'
' get a degree symbol from the translation table or 
' by ascii code to avoid multi-byte character problems
' *******************************************************
FUNCTION GetDegreeSym()
  deg = GetVar("TransDegreeSym")
  if IsEmpty(deg) then
      deg = application.TranslateStr("degreesym")
      if (StrComp(deg, "degreesym") = 0) then
          deg = ""
      end if
      if ( IsBlankString(deg) ) then
        deg = Chr(176)
      end if   
      call SetVar("TransDegreeSym", deg)
  end if
  GetDegreeSym = deg
END FUNCTION


' *******************************************************
' ShowPopupCalendar()
'
' Show popup calendar. Uses the "RunTagCmd ShowCalendar" 
' if available, or uses the "Calendar_Popup" subform
'
' *******************************************************
Sub ShowPopupCalendar(DateTextField, LocationControl)

    
    if GetVersion() >= 10000 then
        
        call Application.RunTagCmd(Form.FormId, LocationControl, "ShowCalendar " & DateTextField)
    else
        '
        ' Get the current value of the text field, and set that value as the initial value for 
        ' the calendar from the text input field
        '
        '
        Call SetVar( "calpopup_pickdate", GetVar(DateTextField) )

        '
        ' Specify the name of the control which should be assigned the selected date
        '
        Call SetVar( "calpopup_returnvar", DateTextField )

        '
        ' Display the popup form
        '
        Application.DoModalForm( "Calendar_Popup" )
    
    end if
    

End Sub

' *******************************************************
' GetUTCTime()
'
' Show popup calendar. Uses the "RunTagCmd ShowCalendar" 
' if available, or uses the "Calendar_Popup" subform
'
' *******************************************************
Function GetUTCTime()
    LocalTime = now()

    ActiveTimeBiasRegKey = "HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\TimeZoneInformation\ActiveTimeBias"
    
    Offset = 0
    
    Set OShell = CreateObject("WScript.Shell")
    On Error Resume Next
    Offset = OShell.RegRead(ActiveTimeBiasRegKey)
    On Error Goto 0
    
    GetUTCTIme = DateAdd("n", Offset, LocalTime)
    
End Function

' *******************************************************
' FormatTimeForSQL()
'
' Format time to use in an SQL query. Checks the database type 
' and returns formatted string.
'
' *******************************************************
Function FormatTimeForSQL(TimeVal)
    
    If GetVar("eActiveDbType") = 0 or GetVar("eActiveDbType") = 4 Then
        '
        'SQL Server
        '
        DateEsc = "'"
    Else
        ' 
        'Access
        '
        DateEsc = "#"
    End If

    FormatTimeForSQL = DateEsc & TimeVal & DateEsc
    
End Function

' *******************************************************
' StandardizeUTCTime() 
'
' Takes a UTCTime and removes any region specific formatting from it. 
'
' *******************************************************
Function StandardizeUTCTime(UtcTime)
    
    hr = Hour(UtcTime)
    min = Minute(UtcTime)
    sec = Second(UtcTime)
    
    AMPMStr = "AM"
    if hr >= 12 then
        AMPMStr = "PM"
    end if

    if hr > 12 then
        hr = hr - 12
    end if

	StandardizeUTCTime = FormatDateTime(UtcTime, vbShortDate)  & " " & hr & ":" & min & ":" & sec & " " & AMPMStr
    
End Function

' *******************************************************
' GetUTCTimeForSQL() 
'
' Get UTC time formatted for use in an SQL query. 
' Checks the database type and correctly formats the string.
'
' *******************************************************
Function GetUTCTimeForSQL()
    
    UtcTime = GetUTCTime()
    UtcStandardizedTime = StandardizeUTCTime(UtcTime)    
    GetUTCTimeForSQL = FormatTimeForSQL(UtcStandardizedTime)
    
End Function

' *******************************************************
' Builds a chart label
'

' *******************************************************
function BuildChartLabel(TagName, LabelStr, Ndx)

	ColorStr = GetChartRGBColorText(Ndx)
    RGBColor = GetChartRGBColorInt(Ndx)
	SymbolStr = GetChartSymbolTxt(Ndx, Empty)
	'LineType = GetChartLineType(TestNdx, Empty)
	
	FullLblStr = LabelStr & "-" & ColorStr & " " & SymbolStr
    
    call SetNdxVar(TagName, Ndx, FullLblStr)

    call Application.RunTagCmd(Form.FormId, TagName & "_"& CStr(Ndx), "FgColor   " + CStr(RGBColor) )    
  
  
End Function

' *******************************************************
' GetChartRGBColorText() 
'
' *******************************************************
Function GetChartRGBColorText(Index)
    ' Color Definitions
    ChartColors = Array(   "Blue",   "Red",     "Green",    "Purple", _
						  "Cyan",    "Yellow",   "Magenta",  "Violet",_
						  "Orange",  "Brown",    "Coral",    "Maroon", _
						  "Olive",   "Teal",     "Sea Green", "Lime",_
						  "Lt. Green", "Midnight Blue", "Sky Blue",      "Beige",_
						  "Gray",  "Gray",   "Gray",   "Gray",_
						  "Gray",  "Gray",   "Gray",   "Gray",	_     
						  "Gray",  "Lt. Red",    "Black" )
 
    if GetVar("NoYellowTraces")  = 1 then 
        ChartColors(5) = "Black" 
    end if
    
    if GetVar("NoGreenTraces")  = 1 then 
        ChartColors(2) = "Olive" 
    end if 
 
    GetChartRGBColorText = ChartColors( ( index - 1 ) Mod 31) 
End Function


' *******************************************************
' GetChartSymbolTxt() 
'
' *******************************************************
Function GetChartSymbolTxt(Index, Default)
    Dim SelectedSymbol
    Symbols = Array("Circle","Triangle","Triangle","Square","Diamond")

    If ( index > 5 ) Then
	If IsEmpty ( Default )  Then
	    SelectedSymbol = Symbols( ( index - 1 ) Mod 5)
	Else
	    SelectedSymbol = Default
	End If
    Else
	SelectedSymbol = Symbols( index - 1 )
    End If

    GetChartSymbolTxt = SelectedSymbol
End Function



' *******************************************************
' GetChartRGBColorInt() 
'
' *******************************************************
Function GetChartRGBColorInt(Index)
    ' Color Definitions
    ChartColors = Array(   CLR_BLUE,   CLR_RED,     CLR_GREEN,   CLR_PURPLE, _
						 CLR_CYAN,    CLR_YELLOW,   CLR_MAGENTA,  CLR_VIOLET,_
						  CLR_ORANGE,  CLR_BROWN,   CLR_CORAL,    CLR_MAROON, _
						 CLR_OLIVE,   CLR_TEAL,     CLR_SEAGREEN, CLR_LIME,_
						  CLR_LTGREEN, CLR_MIDNIGHT, CLR_SKY,     CLR_BEIGE,_
						  CLR_GRAY10,  CLR_GRAY20,   CLR_GRAY30,   CLR_GRAY40,_
						  CLR_GRAY50,  CLR_GRAY60,  CLR_GRAY70,   CLR_GRAY80,	_     
						  CLR_GRAY90,  CLR_LTRED,   CLR_BLACK )
    
    if GetVar("NoYellowTraces") = 1 then 
        ChartColors(5) = CLR_BLACK
    end if
    
    if GetVar("NoGreenTraces")  = 1 then 
        ChartColors(2) = CLR_OLIVE
    end if 
 
    GetChartRGBColorInt = ChartColors( ( index - 1 ) Mod 31) 
End Function

' *******************************************************
' UpdateAsLeftTags(Prefix, TagList) 
'
' Updates the As Left tags by taking a list of As Found tags
' and the prefix of the As Left subform
'
' *******************************************************

function UpdateAsLeftTags(Prefix, TagList)
    
    AFTags = Split(TagList, ",")
    
    for each CurrentTag in AFTags
        if CurrentTag <> empty then
        AFTagName = Split(CurrentTag, "__")    
            if AfTagName(1) <> Empty then
                ALTagName = Prefix & "__" & AfTagName(1)
                call Application.UpdateAsFoundAsLeft(Form.formId, CurrentTag, ALTagName)
            end if
        end if
    next
    
end function



'
'#####################################################
'EMBEDDED CONTROL Custom Field Entry
'#####################################################
'
'
'#####################################################
'EMBEDDED CONTROL EHS_Header
'#####################################################
'

'
'
Sub DoOnSelect__t310_l555_r770_b340
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnInitialUpdate_completedby_t240_l230_r1180_b265
'

	With Form

.completedby=.PdbDeviceGUID
	End With

End Sub

'
'
Sub DoOnSelect__t135_l100_r220_b155
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnSelect__t170_l100_r220_b190
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnSelect__t205_l95_r220_b225
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnSelect__t240_l100_r220_b260
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnSelect__t205_l735_r885_b230
'

	With Form

' msgbox "sel"
	End With

End Sub

'
'
Sub DoOnUpdate_Status_t80_l1015_r1175_b105
'

	With Form

SELECT CASE .Status
    CASE "Open":
    .Ctl__Status_t80_l1015_r1175_b105_color = CLR_OLIVE
    EHSState = 0    
    if .PdbUserAccountLevel <= 3 then
        .Ctl__Status_t80_l1015_r1175_b105_list = "Customer Ready,Open" 
    end if    
   
    CASE "Customer Ready":
    .Ctl__Status_t80_l1015_r1175_b105_color = CLR_BLUE
    
    if (GetVar("EHSState") <> 3) then
        
        QueryStr = "SELECT ResultsGUID FROM Results_Header WHERE JobGUID=" & GetVarForSql("PdbJobGUID")
        Call Application.QueryDatabase(Form.formId, QueryStr, 0)
        if NOT IsBlankString(GetVar("db_ResultsGUID")) then
            ResultsGUID = GetVar("db_ResultsGUID")
            SQL = "SELECT TOP 1 DeltaGUID, DeltaDate FROM ResultsDelta WHERE ResultsGUID = '" & ResultsGUID & "' ORDER BY DeltaDate DESC"
            Call Application.QueryDatabase(Form.formId, SQL, 0)
            
            SqlStr = "UPDATE PdbJob SET Info5 = " & GetVarForSql("db_DeltaGUID") &_
                  " WHERE JobGUID = " & GetVarForSql("PdbJobGUID")
            Call Application.ExecuteSQL(SqlStr)
            EHSState = 3
            if .PdbUserAccountLevel <= 3 then
                .Ctl__Status_t80_l1015_r1175_b105_list = "Customer Ready,Open" 
            end if 
         end if
    end if
    CASE "Closed":
    .Ctl__Status_t80_l1015_r1175_b105_color = CLR_BLACK
    if .PdbUserAccountLevel <= 3 then
        .Ctl__Status_t80_l1015_r1175_b105_readonly = true
    end if 
    EHSState = 4
    CASE ELSE:
    .Ctl__Status_t80_l1015_r1175_b105_color = CLR_MAROON
    if .PdbUserAccountLevel <= 3 then
        .Ctl__Status_t80_l1015_r1175_b105_readonly = true
    end if 
    EHSState = -1
END SELECT

If Form.FormLoadedValues AND GetVar("Info3Check") <> true then
    if NOT IsBlankString(.PdbJobInfo3) then .Status = .PdbJobInfo3
    call SetVar("Info3Check",true)
end if

.PdbJobInfo3  = .Status
    
call SetVar("EHSState", EHSState)
        
	End With

End Sub

'
'
Sub DoOnEnter_Status_t80_l1015_r1175_b105
'

	With Form

tagsarray="conf1,conf2,conf3,conf4"
CanChangeStatus=requiredtags(tagsarray)

if CanChangeStatus=false AND (.Status = "Customer Ready" OR .Status = "Closed") then
      .Status = "Open"
end if
	End With

End Sub

'
'
Sub DoOnInitialUpdate_SiteID_t0_l1210_r1270_b20
'

	With Form

.Ctl__SiteID_t0_l1210_r1270_b20_visible=false
	End With

End Sub

'
'
Sub DoOnInitialUpdate_email_list_t35_l1210_r1275_b55
'

	With Form

.Ctl__email_list_t35_l1210_r1275_b55_visible=false
	End With

End Sub
'
'#####################################################
'EMBEDDED CONTROL EHS_Preparation
'#####################################################
'

'
'
Sub DoOnSelect_other_t750_l100_r120_b770
'

	With Form

call showhide(.other, "describe", "describelbl")
	End With

End Sub

'
'
Sub DoOnSelect_attachemrgplan_t1080_l695_r770_b1100
'

	With Form

MY_DOCUMENTS = &H5&  ' the second & denotes a long integer '

Set objShell = CreateObject("Shell.Application")
Set objFolder = objShell.Namespace(MY_DOCUMENTS)

Set objFolderItem = objFolder.Self

sIniDir = objFolderItem.Path & "\*"
sFilter = "All files (*.*)|*.*|"
sTitle = "Select Parameter File"

' (sIniDir + sFilter + sTitle) size doesn't exceed 191 chars (227 for GetFileDlgBar)
' MsgBox Len(Replace(sIniDir,"\","\\")) + Len(sFilter) + Len(sTitle)

' sIniDir must be conformed to the javascript syntax
FilePath = GetFileDlg(Replace(sIniDir, "\", "\\"), sFilter, sTitle)

if Len(FilePath) > 0 then
    Set objFileToRead = CreateObject("Scripting.FileSystemObject").OpenTextFile(FilePath, 1)
    '
    ' get the entire file as a text string
    '
    Call SetVar("plan",objFileToRead)
    FileContents = objFileToRead.ReadAll()
    objFileToRead.Close
    Set objFileToRead = Nothing

    Call SetVar("ParameterFilePath_" & RecordNdx, FilePath)
    Call SetVar("ParameterFileContents_" & RecordNdx, FileContents)
    
    Call Application.AddSavedDataVar(.FormId, "ParameterFilePath_" & RecordNdx)
    Call Application.AddSavedDataVar(.FormId, "ParameterFileContents_" & RecordNdx)
    Call Application.ImgTagFromFile ("emrg_plan", FilePath)
    MsgBox("Emergency plan attached")
end if
	End With

End Sub

'
'
Sub DoOnSelect__t1080_l800_r875_b1100
'

	With Form

call Application.ChangePage (.FormId, 4, 0)
	End With

End Sub
Sub DoUpdateScriptlet
if GetVar("WaitOnInitialize") = 1 then exit sub 
if GetVar("FirstOnInitDone") = 1 then 
TimeUpdate = GetVar("TimeUpdate") 
FuncDebug = GetVar("FuncDebug") 
if len(FuncDebug) then TimeUpdate = 1 
if TimeUpdate then call alog("StartUpdate")
    CALL PreOnUpdate()
Form.FormMaxCalcDepth = 2 
for NumUpdates = 1 to 2 
 Form.FormCurCalcDepth = NumUpdates

	if TimeUpdate then
		Call alog( "DoOnUpdateModel")
	end if

	if GetVar("TimeScriptFunctions") then
		StartTime = Timer
	end if

	Call DoOnUpdateModel
	if GetVar("TimeScriptFunctions") then
		EndTime = Timer
		if GetVar( "DebugTimerThreshold" ) < EndTime - StartTime then
			alog( "DoOnUpdateModel run time was: " & EndTime - StartTime )
		end if
	end if
    if Len( FuncDebug ) then execute FuncDebug 

	if TimeUpdate then
		Call alog( "DoOnUpdate_User_Data___User_Data_t0_l0_r40_b40")
	end if

	if GetVar("TimeScriptFunctions") then
		StartTime = Timer
	end if

	Call DoOnUpdate_User_Data___User_Data_t0_l0_r40_b40
	if GetVar("TimeScriptFunctions") then
		EndTime = Timer
		if GetVar( "DebugTimerThreshold" ) < EndTime - StartTime then
			alog( "DoOnUpdate_User_Data___User_Data_t0_l0_r40_b40 run time was: " & EndTime - StartTime )
		end if
	end if
    if Len( FuncDebug ) then execute FuncDebug 

	if TimeUpdate then
		Call alog( "DoOnUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16")
	end if

	if GetVar("TimeScriptFunctions") then
		StartTime = Timer
	end if

	Call DoOnUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16
	if GetVar("TimeScriptFunctions") then
		EndTime = Timer
		if GetVar( "DebugTimerThreshold" ) < EndTime - StartTime then
			alog( "DoOnUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16 run time was: " & EndTime - StartTime )
		end if
	end if
    if Len( FuncDebug ) then execute FuncDebug 

	if TimeUpdate then
		Call alog( "DoOnUpdate_Status_t80_l1015_r1175_b105")
	end if

	if GetVar("TimeScriptFunctions") then
		StartTime = Timer
	end if

	Call DoOnUpdate_Status_t80_l1015_r1175_b105
	if GetVar("TimeScriptFunctions") then
		EndTime = Timer
		if GetVar( "DebugTimerThreshold" ) < EndTime - StartTime then
			alog( "DoOnUpdate_Status_t80_l1015_r1175_b105 run time was: " & EndTime - StartTime )
		end if
	end if
    if Len( FuncDebug ) then execute FuncDebug 
Next
    CALL PostOnUpdate2()
    if Len( GetVar("PostOnUpdateFunc" ) ) then execute Form.PostOnUpdateFunc 
if TimeUpdate then call alog("EndUpdate")
end if
End Sub
Sub DoInitUpdateScriptlet
call SetVar("FirstOnInitDone",0)

	Call DoOnInitialUpdateModel

	Call DoOnInitialUpdate_Optima_Data___Do_Not_Change___Optima_Data___Do_Not_Change_t0_l0_r16_b16

	Call DoOnInitialUpdate_completedby_t240_l230_r1180_b265

	Call DoOnInitialUpdate_SiteID_t0_l1210_r1270_b20

	Call DoOnInitialUpdate_email_list_t35_l1210_r1275_b55
	call SetVar("FirstOnInitDone",1) 
	Call DoUpdateScriptlet
End Sub
Sub DoAllOnEnterScriptlet
End Sub
